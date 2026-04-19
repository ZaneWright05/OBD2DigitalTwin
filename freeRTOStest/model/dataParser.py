import os
import threading
from collections import defaultdict, deque
import time
import joblib
from model.gear_estimate import GearEstimator
from model.metricAnalyser import MetricAnalyser, Metrics, Event, TempAnalyser
from model.helpers import ThermalPoint, pid, PIDS, COMPUTEDPIDS, FUELCONSPID, calcInstFuelCons, MetricPoint, TelemetrySnapshot
from IO.input import attempt_serial_connection, read_csv, read_packet, create_log_file
from enum import Enum, auto

class ConnectionState(Enum):
    DISCONNECTED = auto()
    CONNECTED = auto()
    ACTIVE_TRIP = auto()

class Parser:
    def __init__(self):
        self.lock = threading.Lock()
        self.worker = None # thread to get data

        self.filePath = None
        self.logFile = None

        self.state = ConnectionState.DISCONNECTED
        self.serial = None
        self.stop = threading.Event()

        self.tripStartTime = None # for the timer
        self.distanceTravelled_km = 0.0

        if os.path.exists("historicMetrics.joblib"):
            try:
                historicMetrics = joblib.load("historicMetrics.joblib")
            except Exception as e:
                print(f"Error loading historic metrics: {e}, suggest deleting the historic data")
                historicMetrics = None
        else:
            print("No historic metrics found.")
            historicMetrics = None

        if historicMetrics is not None:
            for pid in PIDS.keys():
                if pid in historicMetrics:
                    print(f"Loaded historic metrics for {PIDS[pid].name}: {historicMetrics[pid]}")
                else:
                    print(f"No historic metrics found for {PIDS[pid].name}.")
            for pid in COMPUTEDPIDS.keys():
                if pid in historicMetrics:
                    print(f"Loaded historic metrics for {COMPUTEDPIDS[pid].name}: {historicMetrics[pid]}")
                else:
                    print(f"No historic metrics found for {COMPUTEDPIDS[pid].name}.")

        hist = historicMetrics or {}

        self.rpmMetric = MetricAnalyser(
            PIDS["0x0C"], highThreshold=3000, lowThreshold=500, window_size=9,
            historicMetrics=hist.get("0x0C"), eventsTracked=[True, True, False, False], lowRocThreshold= -200, highRocThreshold=200
        )

        self.speedMetric = MetricAnalyser(
            PIDS["0x0D"], window_size=6, historicMetrics=hist.get("0x0D"),
            conversionFactor=3.6, rocMin=0.5, lowRocThreshold=-3.5, highRocThreshold=2.5, eventsTracked=[False, False, True, True], allowZero=True
        )

        self.loadMetric = MetricAnalyser(
            PIDS["0x04"], window_size=6, historicMetrics=hist.get("0x04"), eventsTracked=[False, False, False, False],
            allowZero=True
        )

        self.throttleMetric = MetricAnalyser(
            PIDS["0x11"], window_size=6, historicMetrics=hist.get("0x11"), eventsTracked=[False, False, False, False], allowZero=True
        )
        # self.loadEstimator = EngineLoadEstimator()

        self.tempMetric = TempAnalyser(PIDS["0x05"], window_size=30, # temp changes slowly, larger window to capture trends ~15s
                                       historicMetrics=hist.get("0x05"), eventsTracked=[False, False, False, False],
                                         highThreshold=105, lowThreshold=75, thresholdTemp=80 , useZScore=True)

        self.airIntakeTempMetric = MetricAnalyser(PIDS["0x0F"], historicMetrics=hist.get("0x0F"), eventsTracked=[True, False, False, False], useZScore=True)

        self.fuelConsMetric = MetricAnalyser(
            COMPUTEDPIDS[FUELCONSPID], historicMetrics=hist.get(FUELCONSPID),
            eventsTracked=[False, False, False, False], window_size=10
        )

        self.MAFMetric = MetricAnalyser(PIDS["0x10"], historicMetrics=hist.get("0x10"), eventsTracked=[False, False, False, False])

        self.voltMetric = MetricAnalyser(PIDS["0x42"], window_size=20, historicMetrics=hist.get("0x42"), eventsTracked=[True, True, True, True], useZScore=False, highThreshold=15.0, lowThreshold=11.8, rocMin=0.4, lowRocThreshold=-1.5, highRocThreshold=1.5)

        self.fuelRailPresMetric = MetricAnalyser(PIDS["0x23"], window_size=20, historicMetrics=hist.get("0x23"), eventsTracked=[True, True, False, False], useZScore=True)

        self.metrics = {
            "0x0C": self.rpmMetric,
            "0x0D": self.speedMetric,
            "0x04": self.loadMetric,
            "0x11": self.throttleMetric,
            "0x05": self.tempMetric,
            "0x0F": self.airIntakeTempMetric,
            "0x10": self.MAFMetric,
            "0x42": self.voltMetric,
            "0x23": self.fuelRailPresMetric,
            FUELCONSPID: self.fuelConsMetric}
           
        self.lastEvent = None
        self.lastEventEndTime = 0 
        self.displayTime = 4 # in seconds
        self.eventsStored = {}
        self.maxEvents = 5


        self.fuelCons = None
        
        self.gearEstimator = GearEstimator()
        self.currentGear = (None, None)
        self.currentGRatio = None


    def save_HistoricMetrics(self):
        if self.tripStartTime is None or self.serial is None:
            return
        if (time.monotonic() - self.tripStartTime < 300):
            print("Trip too short (< 5 minutes), not saving historic metrics...")
            return
        history_payload = {
            pid: metric.update_HistoricMetrics()
            for pid, metric in self.metrics.items()
        }
        joblib.dump(history_payload, "historicMetrics.joblib")
        print("Historic metrics saved.")

    def estimate_gear(self, speed, rpm, throttle):
        if(speed is None or rpm is None):
            return 0
        if speed < 3:
            # low speed we assume neutral
            return 0
    
        self.currentGRatio = rpm / speed
        gear, conf = self.gearEstimator.predict(rpm, speed, throttle)

        if gear is None or conf is None:
            return self.currentGear[0]
        
        currentGear = self.currentGear[0] if self.currentGear[0] is not None else 0

        if conf < 0.25:
            return currentGear
        
        if abs(gear - currentGear) > 1 and conf < 0.5: # require higher confidence for large jumps
            return currentGear
        
        if currentGear == 0 and speed >= 5: # avoid neutral stuck
            return gear
        
        return gear

    def process_packet(self, pid, data0, data1, seq):
        if PIDS.get(pid) is None:
            print(f"Unknown PID: {pid}")
            return
        
        value = PIDS[pid].func(data0, data1)

        with self.lock:
            if pid == "0x0D": 
                rpmVal = self.rpmMetric.metrics.current
                if value != 0.00:
                    print(f"Trip average ROC {self.speedMetric.single_trip_roc_average():.2f}")
                if rpmVal != 0.00:
                    self.currentGear = (self.estimate_gear(value, rpmVal, self.throttleMetric.metrics.current), seq)
                prevSpeed = self.speedMetric.metrics.current
                self.speedMetric.add_data_point(seq, value)

                if prevSpeed is not None:
                    timeDiff = PIDS[pid].period_ms / 1000.0
                    averageSpeed = (prevSpeed + value/3.6) / 2 
                    self.distanceTravelled_km += (averageSpeed * timeDiff) / 1000.0
    
            elif pid == "0x0C":
                self.rpmMetric.add_data_point(seq, value)
                speedVal = self.speedMetric.metrics.current
                if speedVal != 0.00:
                    self.currentGear = (self.estimate_gear(speedVal * 3.6, value, self.throttleMetric.metrics.current), seq) # store timestamp
    
            elif pid == "0x10": # derive inst fuel cons
                self.MAFMetric.add_data_point(seq, value)
                self.fuelConsMetric.add_data_point(seq, calcInstFuelCons(self.speedMetric.metrics.current * 3.6, self.speedMetric.recentSeq, value, seq, self.loadMetric.metrics.current, self.loadMetric.recentSeq))
        
            else:
                metric = self.metrics.get(pid)
                if metric is not None:
                    metric.add_data_point(seq, value)

    def store_gear(self, gear: int):
        with self.lock:
            speed = self.speedMetric.metrics.current * 3.6
            rpm = self.rpmMetric.metrics.current
            throttle = self.throttleMetric.metrics.current

            with open("gear_estim.csv", "a") as f:
                f.write(f"{rpm},{speed},{throttle},{gear}\n")
                f.flush()
            self.gearEstimator.add_data_point(rpm, speed, gear, throttle)

    def calculate_freshness(self):
        score = 1.0
        for metric in self.metrics.values():
            if metric.outOfSequence:
                score -= 1/len(self.metrics) # each out of sequence metric reduces freshness by equal amount
        return max(score, 0.0)

    def get_sorted_events(self):
        events = []
        for metric in self.metrics.values():
            if metric.active_events:
                events.extend(metric.active_events.values())
            
        sorted_events = sorted(events, key=lambda event: (event.timestamp * event.pid.period_ms, event.priority))
        return sorted_events

    def update_events(self, new_events):
        for event in new_events:
            key = (event.pid, event.timestamp)
            if key not in self.eventsStored:
                if len(self.eventsStored) >= self.maxEvents:
                    oldest_key = min(self.eventsStored, key=lambda k: self.eventsStored[k].timestamp * self.eventsStored[k].pid.period_ms)
                    del self.eventsStored[oldest_key]
                self.eventsStored[key] = event

    def get_most_recent(self):
        if not self.connected:
            return None
        
        with self.lock:
            if self.tripStartTime is None:
                self.tripStartTime = time.monotonic()
            elapsedTime_s = time.monotonic() - self.tripStartTime  
            hours = int(elapsedTime_s // 3600)
            minutes = int((elapsedTime_s % 3600) // 60)
            seconds = int(elapsedTime_s % 60)
            time_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"

            sortedEvents = self.get_sorted_events()
            self.update_events(sortedEvents)
            if sortedEvents:
                recentEvent = sortedEvents[-1]
            else:
                recentEvent = None

            if recentEvent is not None:
                self.lastEvent = recentEvent
                self.lastEventEndTime = 0.0
            else:
                now = time.monotonic()
                if self.lastEvent and self.lastEventEndTime == 0.0:
                    self.lastEventEndTime = now
                if self.lastEvent and (now - self.lastEventEndTime) < self.displayTime:
                    recentEvent = self.lastEvent
                else:
                    self.lastEvent = None
                    self.lastEventEndTime = 0.0
            
            # print(self.rpmMetric.metrics.wAvgROC)

            return {
                "time": time_str,
                "distance": f"{self.distanceTravelled_km:.2f}",
                "rpm" : self.rpmMetric,
                "speed": self.speedMetric,
                "temp": self.tempMetric,
                "volt": self.voltMetric,
                "load": self.loadMetric,
                "throttle": self.throttleMetric,
                "event": recentEvent,
                "allEvents": self.eventsStored,
                "fuelCons": self.fuelConsMetric,  
                "gear": self.currentGear[0],
                "ratio": self.currentGRatio,
                "freshness": self.calculate_freshness()
            }
    
    def createMetricPoint(self, metric: MetricAnalyser):
        if not metric.active_events and metric.active_events != {}:
            priorities = [event.priority for event in metric.active_events.values()]
            highestPri = max(priorities)
            eventCount = len(priorities)
        else:
            highestPri = None
            eventCount = 0

        m = metric.metrics
        hist = metric.historicMetrics

        if isinstance(metric, TempAnalyser):
            return ThermalPoint(
                current=m.current if m else None,
                average=m.average if m else None,
                wAvgROC=m.wAvgROC if m else None,
                max=m.max if m else None,
                min=m.min if m else None,
                outOfSequence=metric.outOfSequence,
                eventCount=eventCount,
                highestPriority=highestPri,

                ## historic metrics
                allTripAverage=metric.all_trip_average(),
                allTripMin=hist.min if hist else None,
                allTripMax=hist.max if hist else None,
                allTripAverageROC=metric.all_trip_wAvgROC() if hist else None,
                tripCount=hist.tripCount if hist else 0,

                ## temp specific
                coolantThreshold=metric.thresholdTemp,
                overheatThreshold=metric.highThreshold,
                tempLowThreshold=metric.lowThreshold
                )
       

        return MetricPoint(
            current=m.current if m else None,
            average=m.average if m else None,
            wAvgROC=m.wAvgROC if m else None,
            max=m.max if m else None,
            min=m.min if m else None,
            outOfSequence=metric.outOfSequence,
            eventCount=eventCount,
            highestPriority=highestPri,

            ## historic metrics
            allTripAverage=metric.all_trip_average(),
            allTripMin=hist.min if hist else None,
            allTripMax=hist.max if hist else None,
            allTripAverageROC=metric.all_trip_wAvgROC() if hist else None,
            tripCount=hist.tripCount if hist else 0
            )
    
    def get_snapshot(self) -> TelemetrySnapshot:
        with self.lock:
            metricPoints = {pid: self.createMetricPoint(metric) for pid, 
                            metric in self.metrics.items()
                            }
            
            return TelemetrySnapshot(
                trip_time_s=(time.monotonic() - self.tripStartTime) if self.tripStartTime else 0.0,
                trip_distance_km=self.distanceTravelled_km,
                freshness=self.calculate_freshness(),
                current_gear=self.currentGear[0],
                gear_ratio=self.currentGRatio,
                metrics=metricPoints
            )
    
    @property
    def connected(self):
        if self.serial is not None:
            return self.serial.is_open
        return self.state != ConnectionState.DISCONNECTED

    @property
    def running(self):
        return self.state == ConnectionState.ACTIVE_TRIP

    def reset_trip(self):
        with self.lock:
            for metric in self.metrics.values():
                metric.reset_runtime()

            self.currentGear = (None, None)
            self.currentGRatio = None
            self.lastEvent = None
            self.lastEventEndTime = 0.0
            self.tripStartTime = time.monotonic()
            self.distanceTravelled_km = 0.0
            self.eventsStored = {}

            self.filePath = None
            self.logFile = None

    def start_trip(self):
        if self.state != ConnectionState.CONNECTED or not self.connected:
            return False
        
        self.reset_trip()
        if self.serial is not None:
            try:
                self.serial.write(b"S\n")
                self.serial.flush()
            except Exception as e:
                print(f"Exception while sending start command: {e}")
                self.state = ConnectionState.DISCONNECTED
                return False
            self.filePath = create_log_file()
            self.logFile = open(self.filePath, "w")
            self.logFile.write("PID,Data0,Data1,Seq\n")
            self.logFile.flush()

        self.state = ConnectionState.ACTIVE_TRIP
        return True

    def stop_trip(self):
        if self.state != ConnectionState.ACTIVE_TRIP:
            return False
        
        if self.serial is not None and self.connected:
            try:
                self.serial.write(b"X\n")
                self.serial.flush()
                self.state = ConnectionState.DISCONNECTED
                # time.sleep(4) # wait for final packets
            except Exception as e: 
                self.state = ConnectionState.DISCONNECTED
                return False
        else:
            self.state = ConnectionState.CONNECTED
        self.save_HistoricMetrics()
        return True

    def run_com_loop(self):
        # fileCreated = False
        while self.connected and not self.stop.is_set():
            try:
                packet = read_packet(self.serial)
            except Exception as e:
                print(f"Serial disconnected: {e}")
                self.state = ConnectionState.DISCONNECTED
                return "disconnected"
            
            if packet is None:
                continue
            
            pid, data0, data1, seq = packet
            if self.logFile:
                try:
                    print("Logging packet to file:", pid, data0, data1, seq)
                    self.logFile.write(f"{pid},{data0:02X},{data1:02X},{seq}\n")
                    self.logFile.flush()
                except Exception as e:
                    print(f"Error writing to log file: {e}")

            self.process_packet(pid, data0, data1, seq)
        return "stopped"


    def select_mode(self, mode="serial", csv_path="", sample_rate=64):
        if mode != "serial":
            self.state = ConnectionState.CONNECTED
            while not self.stop.is_set():
                if not self.running:
                    time.sleep(0.05)
                    continue
                read_csv(csv_path, self, sample_rate)   # replay once per trip
                if self.state == ConnectionState.ACTIVE_TRIP:
                    self.state = ConnectionState.CONNECTED
            self.state = ConnectionState.DISCONNECTED
            return

        while not self.stop.is_set():
            self.serial = attempt_serial_connection()
            if self.serial is None:
                print("No serial connection.")
                self.state = ConnectionState.DISCONNECTED
                time.sleep(1)
                continue
            self.state = ConnectionState.CONNECTED
            self.run_com_loop()

            try:
                if self.serial is not None and self.serial.is_open:
                    self.serial.close()
            except Exception:
                pass

            self.serial = None
            self.state = ConnectionState.DISCONNECTED
            time.sleep(0.5)

    def start_parsing(self, mode="serial", csv_path="", sample_rate=64):
        if self.worker is not None and self.worker.is_alive():
            print("Data parsing is already running.")
            return
        self.worker = threading.Thread(
            target=self.select_mode,
            kwargs={
                "mode": mode,
                "csv_path": csv_path,
                "sample_rate": sample_rate
            },
            daemon=True
        )
        self.worker.start()