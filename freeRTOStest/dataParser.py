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
from metricAnalyser import MetricAnalyser, Metrics, Event
from helpers import pid

PIDS = {
    # need to add engine load to the design
    "0x0C": pid(2, "rpm", lambda a,b: ((a * 256) + b )/ 4, 333),
    "0x0D": pid(1, "kmh", lambda a,b: a, 333),

    "0x11": pid(1, "%", lambda a,b: (a * 100)/255, 500),
    "0x0F": pid(1, "C", lambda a,b : a - 40, 500),

    ## might be included in data?
    "0x04": pid(1, "%", lambda a,b: (a * 100)/255, 500),

    "0x10": pid(2, "g/s", lambda a,b : ((256 * a) + b)/100,1000),
    "0x42": pid(2, "V", lambda a,b: ((256* a) + b)/1000, 1000),
    
    "0x05": pid(1, "C", lambda a,b : a - 40,2000),
    "0x23": pid(2, "kPa", lambda a,b : 10*((256*a) + b),2000),

    "0x1F": pid(2, "s", lambda a, b: (256 * a) + b,4000)
    }


FUELCONSPID = "0xFF"
COMPUTEDPIDS = {
    "0xFF" : pid(2, "l/100km", lambda speed, speedSeq, maf, mafSeq: calcInstFuelCons(speed, speedSeq, maf, mafSeq), max(PIDS["0x0D"].period_ms, PIDS["0x10"].period_ms, PIDS["0x04"].period_ms))
}

# @dataclass(frozen=True)
# class computedMetric:
#     name: str
#     unit: str
#     parentPid: pid
#     func: callable
#     # timestamp_ms: int

# computedMetrics = {
#     "acc" : computedMetric("acceleration", "m/s^2", PIDS["0x0D"], lambda window, pid: savgol_filter(window, pid))
# }

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
            historicMetrics = joblib.load("historicMetrics.joblib")
        else:
            historicMetrics = {}

        if "0x0C" in historicMetrics:
            print(f"Loaded historic metrics for RPM: {historicMetrics['0x0C']}")
        else:
            print("No historic metrics found for RPM.")
        self.rpmMetric = MetricAnalyser(PIDS["0x0C"], highThreshold=6000, lowThreshold=500, rocMin=0.5,window_size=15, historicMetrics=historicMetrics.get("0x0C"))
        # self.rpmMetric.parent = self

        self.speedMetric = MetricAnalyser(PIDS["0x0D"], window_size=5, historicMetrics=historicMetrics.get("0x0D"), conversionFactor=3.6)

        if FUELCONSPID in historicMetrics:
            print(f"Loaded historic metrics for fuel consumption: {historicMetrics[FUELCONSPID]}")
        else:
            print("No historic metrics found for fuel cons")
        self.fuelConsMetric = MetricAnalyser(COMPUTEDPIDS[FUELCONSPID], historicMetrics=historicMetrics.get(FUELCONSPID), eventsTracked = False)

        self.lastEvent = None
        self.lastEventEndTime = 0 
        self.displayTime = 2 # in seconds
        
        self.fuelCons = None
        
        self.gearEstimator = GearEstimator()
        self.currentGear = (None, None)
        self.currentGRatio = None

        self.windowSize = 5
        self.polyNom = 2
        self.speedWindow = deque(maxlen=self.windowSize)
        # self.timeWindow = deque(maxlen=self.windowSize)

    def save_HistoricMetrics(self):
        # save historic metrics to file
        if (time.monotonic() - self.connectionStartTime < 300):
            print("Trip too short (< 5 minutes), not saving historic metrics...")
            return
        joblib.dump({
            "0x0C": self.rpmMetric.update_HistoricMetrics(),
            FUELCONSPID: self.fuelConsMetric.update_HistoricMetrics()
        }, "historicMetrics.joblib")
        print(f"{self.rpmMetric.historicMetrics}\n{self.fuelConsMetric.historicMetrics}")

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
                rpm_data, rpm_seq = self.mostRecentValues.get("0x0C", (None, None))
                if rpm_data is not None and seq == rpm_seq and self.currentGear[1] != seq:
                    self.currentGear = (self.estimate_gear(value, rpm_data), seq)
                prevSpeed = self.speedMetric.metrics.current if self.speedMetric.metrics else None
                self.speedMetric.add_data_point(seq, value)
                if prevSpeed is not None:
                    timeDiff = PIDS[pid].period_ms / 1000.0
                    averageSpeed = (prevSpeed + value) / 2
                    self.distanceTravelled_km += (averageSpeed * timeDiff) / 3600.0
    
            if pid == "0x0C":
                self.rpmMetric.add_data_point(seq, value)
                speed_data, speed_seq = self.mostRecentValues.get("0x0D", (None, None))
                if speed_data is not None and seq == speed_seq and self.currentGear[1] != seq:
                    # update if same and not already used for gear estimation
                    self.currentGear = (self.estimate_gear(speed_data, value), seq) # store timestamp
                # self.currentGear = self.estimate_gear(self.mostRecentValues.get("0x0D")[0], value)

            # print(f"PID: {pid}, Value: {value} {PIDS[pid].unit}, Seq: {seq}")
    
            if pid == "0x10": # derive inst fuel cons
                speed, speedSeq = self.mostRecentValues.get("0x0D")
                load, loadSeq = self.mostRecentValues.get("0x04")
                self.fuelConsMetric.add_data_point(seq, calcInstFuelCons(speed, speedSeq, value, seq, load, loadSeq))

            self.pidValues[pid].append((value, seq))
            self.mostRecentValues[pid] = (value, seq)

    def store_gear(self, gear: int):
        with self.lock:
            speed, speedSeq = self.mostRecentValues.get("0x0D", (None, None))
            rpm, rpmSeq = self.mostRecentValues.get("0x0C", (None, None))
            if speed is None or rpm is None or speed < 5:
                return
            matching = False
            if speedSeq != rpmSeq:
                if speedSeq > rpmSeq and len(self.pidValues["0x0D"]) >= 2: ## speed is more recent get previous so it matches rpm
                    speedOld, speedOldSeq = self.pidValues["0x0D"][-2]
                    if speedOldSeq == rpmSeq:
                        speed = speedOld
                        matching = True
                elif len(self.pidValues["0x0C"]) >= 2:
                    rpmOld, rpmOldSeq = self.pidValues["0x0C"][-2]
                    if rpmOldSeq == speedSeq:
                        rpm = rpmOld
                        matching = True
            # self.currentGear = gear
            else:
                matching = True


            if not matching:
                print(f"No matching RPM and speed data for gear label, skipping {rpmSeq}, {speedSeq}...")
                return

            with open("gear_estim.csv", "a") as f:
                f.write(f"{self.mostRecentValues.get('0x0C', (None, None))[0]},{self.mostRecentValues.get('0x0D', (None, None))[0]},{self.currentAcc:.2f},{gear}\n")
                f.flush()
            self.gearEstimator.add_data_point(rpm, speed, gear)

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
            if(self.rpmMetric.active_events):
                highestPir = -1  
                for event in self.rpmMetric.active_events.values():
                    if event.priority > highestPir:
                        highestPir = event.priority
                        recentEvent = event
                    elif event.priority == highestPir:
                        if recentEvent is None or event.timestamp > recentEvent.timestamp:
                            recentEvent = event
                self.lastEvent = recentEvent
                self.lastEventEndTime = 0
            else: # handling for no active events but giving user time to read
                if self.lastEvent and self.lastEventEndTime == 0:
                    self.lastEventEndTime = time.time()
                if self.lastEvent and (time.time() - self.lastEventEndTime) < self.displayTime:
                    recentEvent = self.lastEvent
                else:
                    self.lastEvent = None
                    self.lastEventEndTime = 0
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
