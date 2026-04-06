# test script to analyse the data log from recieve.py

import csv
import os
import threading
from collections import defaultdict, deque
import time
# import matplotlib.pyplot as plt
import joblib
from scipy.signal import savgol_filter
from dataclasses import dataclass
#from scipy.signal import savgol_filter
import numpy as np
from time import sleep 
import serial
from serial.tools import list_ports
from datetime import datetime

from gear_estimate import GearEstimator
from metricAnalyser import MetricAnalyser, Metrics, Event, TempAnalyser
from helpers import pid
# from estimateEngineLoad import EngineLoadEstimator

PIDS = {
    # need to add engine load to the design
    "0x0C": pid("RPM", 2, "rpm", lambda a,b: ((a * 256) + b )/ 4, 333),
    "0x0D": pid("Speed", 1, "kmh", lambda a,b: a, 333),

    "0x11": pid("Throttle", 1, "%", lambda a,b: (a * 100)/255, 500),
    "0x05": pid("Engine Coolant Temperature", 1, "C", lambda a,b : a - 40, 500),

    ## might be included in data?
    "0x04": pid("Engine Load", 1, "%", lambda a,b: (a * 100)/255, 500),

    "0x10": pid("Mass Air Flow", 2, "g/s", lambda a,b : ((256 * a) + b)/100,1000),
    "0x42": pid("Battery Voltage", 2, "V", lambda a,b: ((256* a) + b)/1000, 1000),
    
    "0x0F": pid("Intake Air Temperature", 1, "C", lambda a,b : a - 40,2000),
    "0x23": pid("Intake Manifold Pressure", 2, "kPa", lambda a,b : 10*((256*a) + b),2000),

    "0x1F": pid("Engine Runtime", 2, "s", lambda a, b: (256 * a) + b,4000)
    }


FUELCONSPID = "0xFF"
COMPUTEDPIDS = {
    "0xFF" : pid("Instantaneous Fuel Consumption", 2, "l/100km", lambda speed, speedSeq, maf, mafSeq: calcInstFuelCons(speed, speedSeq, maf, mafSeq), max(PIDS["0x0D"].period_ms, PIDS["0x10"].period_ms, PIDS["0x04"].period_ms))
}

# ranges from https://caracaltech.com/articles/article/627981fe2fb15a8d9e50f99c
# lambda afr/afrStoch (14.5 diesel), afr vals from link above
def load_to_lambda(load):
    if load < 100/3: # low load
        lowRange = (6.9, 2.8)
        pos = load / (100/3)
        return lowRange[0] + pos * (lowRange[1] - lowRange[0])
    elif load < 100 * (2/3): # medium load
        medRange = (2.8, 1.7)
        pos = (load - 100/3) / (100 * (2/3) - 100/3)
        return medRange[0] + pos * (medRange[1] - medRange[0])
    else: # highload
        highRange = (1.7, 1.1)
        pos = (load - (100 * (2/3))) / (100 - (100 * (2/3)))
        return highRange[0] + pos * (highRange[1] - highRange[0])

def calcInstFuelCons(speed, speedSeq, maf, mafSeq, load, loadSeq):
    speedTime = speedSeq * PIDS["0x0D"].period_ms / 1000.0
    mafTime = mafSeq * PIDS["0x10"].period_ms / 1000.0
    loadTime = loadSeq * PIDS["0x04"].period_ms / 1000.0
    valid = abs(speedTime - mafTime) < 0.5 and abs(speedTime - loadTime) < 0.5 and abs(mafTime - loadTime) < 0.5
    if valid and speed != 0: # compare timestamps

        ff = (maf * 3600) / (load_to_lambda(load) *14.5 * 820)
        fCons = ff / speed
        return (fCons * 100) # convert to l/100km
    else:
        return None

class Analyser:
    def __init__(self):
        self.lock = threading.Lock()
        self.pidValues = defaultdict(list)
        self.mostRecentValues = {}
        
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

        self.metrics = {
            "0x0C": self.rpmMetric,
            "0x0D": self.speedMetric,
            "0x04": self.loadMetric,
            "0x11": self.throttleMetric,
            "0x05": self.tempMetric,
            "0x10": self.MAFMetric,
            FUELCONSPID: self.fuelConsMetric}
           
        self.lastEvent = None
        self.lastEventEndTime = 0 
        self.displayTime = 4 # in seconds
        
        self.fuelCons = None
        
        self.gearEstimator = GearEstimator()
        self.currentGear = (None, None)
        self.currentGRatio = None

        self.windowSize = 5
        self.polyNom = 2
        self.speedWindow = deque(maxlen=self.windowSize)

        # self.timeWindow = deque(maxlen=self.windowSize)

    def save_HistoricMetrics(self):
        # self.loadEstimator.save_bins()
        # print("Engine load estimator bins saved.")
        # save historic metrics to file
        if (time.monotonic() - self.connectionStartTime < 300):
            print("Trip too short (< 5 minutes), not saving historic metrics...")
            return
        joblib.dump({
            "0x0C": self.rpmMetric.update_HistoricMetrics(),
            "0x0D": self.speedMetric.update_HistoricMetrics(),
            "0x04": self.loadMetric.update_HistoricMetrics(),
            "0x11": self.throttleMetric.update_HistoricMetrics(),
            "0x05": self.tempMetric.update_HistoricMetrics(),
            "0x10": self.MAFMetric.update_HistoricMetrics(),
           FUELCONSPID: self.fuelConsMetric.update_HistoricMetrics()
        }, "historicMetrics.joblib")
        print("Historic metrics saved.")

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

            self.pidValues[pid].append((value, seq))
            self.mostRecentValues[pid] = (value, seq)

    # fix, to use speed ROC not acc
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

    def get_snapshot(self):
        return {
            "metrics": self.metrics,
            "tripLength" : self.distanceTravelled_km,
            "tripTime" : time.monotonic() - self.connectionStartTime if self.connectionStartTime else 0,
            "currentGear": self.currentGear,
            "freshness": self.calculate_freshness()
        }

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
            latestData = {
                pid: {"value": value, "unit": PIDS[pid].unit}
                for pid, (value, seq) in self.mostRecentValues.items()
            }
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
                "latestData": latestData,
                "rpm" : self.rpmMetric,
                "speed": self.speedMetric,
                "event": recentEvent,
                "fuelCons": self.fuelConsMetric,  
                "gear": self.currentGear[0],
                "ratio": self.currentGRatio
            }


def find_connected_port():
    ports = list(list_ports.comports())

    # Best match: Raspberry Pi USB VID (0x2E8A)
    for p in ports:
        if p.vid == 0x2E8A:
            return p.device

    # Fallbacks by device naming
    for p in ports:
        dev = p.device
        if dev.startswith("/dev/ttyACM") or dev.startswith("/dev/ttyUSB"):
            return dev
        if dev.upper().startswith("COM"):
            return dev
    return None

def read_from_com(store):
    print("Attempting to connect to serial port...")

    while not store.connected:
        port = find_connected_port()
        # port = "COM4"
        if port is None:
            print("No serial port found. Retrying in 1 second...")
            sleep(1)
        else:
            ser = serial.Serial(port, 115200)
            print(f"Connected to serial port: {port}")
            store.connected = True
        # try:
        #     ser = serial.Serial('COM4', 115200)
        #     print("Connected to serial port.")
        #     notConnected = False
        # except serial.SerialException:
        #     sleep(1)
    ser.flushInput()
    log_name = f"data_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(log_name, "a") as f:
        f.write("PID,Data0,Data1,Seq\n")
        f.flush()
        while True:
            line = ser.readline()
            if line:
                line = line.decode("utf-8").strip()
                parts = line.split(',')
                if (len(parts) == 4):
                    pid = parts[0]
                    try: 
                        data0 = int(parts[1], 16)
                        data1 = int(parts[2], 16) 
                        seq = int(parts[3].strip(),10)
                        f.write(f"{pid},{data0:02X},{data1:02X},{seq}\n")
                        f.flush()
                        # print(f"PID: {pid}, Data: {data0:02X} {data1:02X}, Seq: {seq}") 
                        store.process_packet(pid, data0, data1, seq)
                    except ValueError:
                        print(f"Invalid data format")
            else:
                print("No data received")

def read_csv(path, store, sample_rate=16):
    delay = 1.0 / sample_rate
    store.connected = True
    with open('data_log_4.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            pid = row['PID']
            data0 = int(row['Data0'], 16)
            data1 = int(row['Data1'], 16)
            seq = int(row['Seq'])
            store.process_packet(pid, data0, data1, seq)
            sleep(delay)
