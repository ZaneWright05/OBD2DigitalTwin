import os
import threading
from collections import defaultdict, deque
import time
import joblib
from model.gear_estimate import GearEstimator
from model.metricAnalyser import MetricAnalyser, Metrics, Event, TempAnalyser
from model.helpers import pid, PIDS, COMPUTEDPIDS, FUELCONSPID, calcInstFuelCons, MetricPoint, TelemetrySnapshot

class Parser:
    def __init__(self):
        self.lock = threading.Lock()
        
        self.connected = False
        self.connectionStartTime = None # for the timer
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
            historicMetrics=hist.get("0x0C"), eventsTracked=[True, True, False, False]
        )

        self.speedMetric = MetricAnalyser(
            PIDS["0x0D"], window_size=6, historicMetrics=hist.get("0x0D"),
            conversionFactor=3.6, rocMin=0.5, lowRocThreshold=-3.5, highRocThreshold=2.5, eventsTracked=[False, False, True, True]
        )

        self.loadMetric = MetricAnalyser(
            PIDS["0x04"], window_size=6, historicMetrics=hist.get("0x04"), eventsTracked=[False, False, False, False]
        )

        self.throttleMetric = MetricAnalyser(
            PIDS["0x11"], window_size=6, historicMetrics=hist.get("0x11"), eventsTracked=[False, False, False, False]
        )
        # self.loadEstimator = EngineLoadEstimator()

        self.tempMetric = TempAnalyser(PIDS["0x05"], historicMetrics=hist.get("0x05"), eventsTracked=[False, False, False, False], highThreshold=105, lowThreshold=85, thresholdTemp=87.5)

        self.fuelConsMetric = MetricAnalyser(
            COMPUTEDPIDS[FUELCONSPID], historicMetrics=hist.get(FUELCONSPID),
            eventsTracked=[False, False, False, False], window_size=10
        )

        self.MAFMetric = MetricAnalyser(PIDS["0x10"], window_size=6, historicMetrics=hist.get("0x10"), eventsTracked=[False, False, False, False])

        self.voltMetric = MetricAnalyser(PIDS["0x42"], window_size=6, historicMetrics=hist.get("0x42"), eventsTracked=[False, False, False, False])

        self.metrics = {
            "0x0C": self.rpmMetric,
            "0x0D": self.speedMetric,
            "0x04": self.loadMetric,
            "0x11": self.throttleMetric,
            "0x05": self.tempMetric,
            "0x10": self.MAFMetric,
            "0x42": self.voltMetric,
            FUELCONSPID: self.fuelConsMetric}
           
        self.lastEvent = None
        self.lastEventEndTime = 0 
        self.displayTime = 4 # in seconds
        
        self.fuelCons = None
        
        self.gearEstimator = GearEstimator()
        self.currentGear = (None, None)
        self.currentGRatio = None

    def save_HistoricMetrics(self):
        if self.connectionStartTime is None:
            return
        if (time.monotonic() - self.connectionStartTime < 300):
            print("Trip too short (< 5 minutes), not saving historic metrics...")
            return
        history_payload = {
            pid: metric.update_HistoricMetrics()
            for pid, metric in self.metrics.items()
        }
        joblib.dump(history_payload, "historicMetrics.joblib")
        print("Historic metrics saved.")

    # TODO: use the confidence and add transition logic, 
    # i.e if currently in gear 3, only transition to gear 2 or 4 and require higher confidence for a jump to gear 1 or 5
    def estimate_gear(self, speed, rpm):
        if(speed is None or rpm is None):
            return 0
        if speed < 5 or rpm < self.rpmMetric.metrics.min * 1.25:
            # low speed or rpm we assume neutral
            return 0
    
        self.currentGRatio = rpm / speed
        gear, conf = self.gearEstimator.predict(rpm, speed)
        if gear is not None and conf is not None:
        # and conf >= 0.25:
            return gear

        return 0

    def process_packet(self, pid, data0, data1, seq):
        if PIDS.get(pid) is None:
            print(f"Unknown PID: {pid}")
            return
        
        value = PIDS[pid].func(data0, data1)

        with self.lock:
            # print(f"Processing PID: {pid}, Value: {value} {PIDS[pid].unit}, Seq: {seq}")
            # for metric in computedMetrics.values():
            if pid == "0x0D":
                # rpm_data, rpm_seq = self.mostRecentValues.get("0x0C", (None, None))
                rpmSeq = self.rpmMetric.recentSeq
                rpmVal = self.rpmMetric.metrics.current
                
                if rpmVal != 0.00 and seq == rpmSeq and self.currentGear[1] != seq:
                    self.currentGear = (self.estimate_gear(value, rpmVal), seq)
                
                prevSpeed = self.speedMetric.metrics.current if self.speedMetric.metrics else None
                self.speedMetric.add_data_point(seq, value)

                if prevSpeed is not None:
                    timeDiff = PIDS[pid].period_ms / 1000.0
                    averageSpeed = (prevSpeed + value) / 2
                    self.distanceTravelled_km += (averageSpeed * timeDiff) / 3600.0
    
            elif pid == "0x0C":
                self.rpmMetric.add_data_point(seq, value)
                speedSeq = self.speedMetric.recentSeq
                speedVal = self.speedMetric.metrics.current
                if speedVal != 0.00 and seq == speedSeq and self.currentGear[1] != seq:
                    self.currentGear = (self.estimate_gear(speedVal, value), seq) # store timestamp
    
            elif pid == "0x10": # derive inst fuel cons
                self.MAFMetric.add_data_point(seq, value)
                
                ## with maf we update load estimator and fuel cons metric
                self.fuelConsMetric.add_data_point(seq, calcInstFuelCons(self.speedMetric.metrics.current, self.speedMetric.recentSeq, value, seq, self.loadMetric.metrics.current, self.loadMetric.recentSeq))
                # self.loadEstimator.update(self.rpmMetric.metrics.current, value, self.speedMetric.metrics.wAvgROC)

            elif pid == "0x04":
                self.loadMetric.add_data_point(seq, value)
                
                # loadEstim = self.loadEstimator.estimate_load(self.rpmMetric.metrics.current, self.MAFMetric.metrics.current)
                # if loadEstim is not None:
                #     print(f"Actual Load: {value:.2f}%, Estimated Load: {loadEstim:.2f}%")
            else:
                metric = self.metrics.get(pid)
                if metric is not None:
                    metric.add_data_point(seq, value)

    def store_gear(self, gear: int):
        with self.lock:
            speed = self.speedMetric.metrics.current
            speedSeq = self.speedMetric.recentSeq
            rpm = self.rpmMetric.metrics.current
            rpmSeq = self.rpmMetric.recentSeq 
            if speed is None or rpm is None or speed < 5:
                return
            matching = False
            if speedSeq != rpmSeq:
                if speedSeq == (rpmSeq - 1): ## rpm is more recent get previous so it matches speed
                    rpm = self.rpmMetric.data[-2][1] if len(self.rpmMetric.data) >= 2 else None
                    matching = True

                elif rpmSeq == (speedSeq - 1): ## speed is more recent get previous so it matches rpm
                    speed = self.speedMetric.data[-2][1] if len(self.speedMetric.data) >= 2 else None
                    matching = True
            else:
                matching = True


            if not matching:
                print(f"No matching RPM and speed data for gear label, skipping {rpmSeq}, {speedSeq}...")
                return

            with open("gear_estim.csv", "a") as f:
                f.write(f"{rpm},{speed},{self.speedMetric.metrics.maxWAvgROC:.2f},{gear}\n")
                f.flush()
            self.gearEstimator.add_data_point(rpm, speed, gear)

    def calculate_freshness(self):
        score = 1.0
        for metric in self.metrics.values():
            if metric.outOfSequence:
                score -= 1/len(self.metrics) # each out of sequence metric reduces freshness by equal amount
        return max(score, 0.0)

    def get_most_recent(self):
        if not self.connected:
            return None
        
        with self.lock:
            if self.connectionStartTime is None:
                self.connectionStartTime = time.monotonic()
            elapsedTime_s = time.monotonic() - self.connectionStartTime  
            hours = int(elapsedTime_s // 3600)
            minutes = int((elapsedTime_s % 3600) // 60)
            seconds = int(elapsedTime_s % 60)
            time_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"


            recentEvent = None
            highestPri = -1
            for metric in self.metrics.values():
                if metric.active_events:
                    for event in metric.active_events.values():
                        if event.priority > highestPri:
                            highestPri = event.priority
                            recentEvent = event
                        elif event.priority == highestPri:
                            if recentEvent is None or event.timestamp > recentEvent.timestamp:
                                recentEvent = event
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
            
            return {
                "time": time_str,
                "distance": f"{self.distanceTravelled_km:.2f}",
                "rpm" : self.rpmMetric,
                "speed": self.speedMetric,
                "temp": self.tempMetric,
                "volt": self.voltMetric,
                "event": recentEvent,
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
        return MetricPoint(
            current=m.current if m else None,
            average=m.average if m else None,
            wAvgROC=m.wAvgROC if m else None,
            max=m.max if m else None,
            min=m.min if m else None,
            outOfSequence=metric.outOfSequence,
            eventCount=eventCount,
            highestPriority=highestPri,
            allTripAverage=metric.all_trip_average()
            )
    
    def get_snapshot(self) -> TelemetrySnapshot:
        with self.lock:
            metricPoints = {pid: self.createMetricPoint(metric) for pid, 
                            metric in self.metrics.items()
                            }
            
            return TelemetrySnapshot(
                trip_time_s=(time.monotonic() - self.connectionStartTime) if self.connectionStartTime else 0.0,
                trip_distance_km=self.distanceTravelled_km,
                freshness=self.calculate_freshness(),
                current_gear=self.currentGear[0],
                gear_ratio=self.currentGRatio,
                metrics=metricPoints
            )