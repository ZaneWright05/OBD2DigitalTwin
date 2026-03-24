# test script to analyse the data log from recieve.py

import csv
import threading
from collections import defaultdict, deque
# import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from dataclasses import dataclass
#from scipy.signal import savgol_filter
import numpy as np
from time import sleep 
import serial
from serial.tools import list_ports
from datetime import datetime

@dataclass(frozen=True)
class pid:
    byte_count: int
    unit: str
    func: callable
    period_ms: int

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

@dataclass(frozen=True)
class computedMetric:
    name: str
    unit: str
    parentPid: pid
    func: callable
    # timestamp_ms: int

computedMetrics = {
    "acc" : computedMetric("acceleration", "m/s^2", PIDS["0x0D"], lambda window, pid: savgol_filter(window, pid))
}

def applySGFilter(window, pid):
    # print(f"Applying SG filter to window: {window}")
                    # self.timeWindow.append(time)

    if len(window) == window.maxlen:
        y = np.array(window, dtype=np.float64)            
        delta = PIDS[pid].period_ms / 1000.0

        dydt = savgol_filter(
            y,
            window_length=window.maxlen,
            polyorder=2,
            deriv=1,
            delta=delta,
            mode="interp" 
            )
        return float(dydt[-1])
    else:
        return None

class Analyser:
    def __init__(self):
        self.lock = threading.Lock()
        self.pidValues = defaultdict(list)
        self.mostRecentValues = {}
        self.currentSpeed = None
        
        self.currentAcc = None
        self.currentAccRaw = None
        
        self.fuelCons = None
        self.currentGear = None

        self.windowSize = 5
        self.polyNom = 2
        self.speedWindow = deque(maxlen=self.windowSize)
        # self.timeWindow = deque(maxlen=self.windowSize)

    def estimate_gear(self, speed, rpm):
        if(speed is None or rpm is None or speed == 0):
            return 0
        ratio = rpm / (speed) # add 1 to avoid division by zero
        return ratio # this is just a place holder, later KNN will be used


    def process_packet(self, pid, data0, data1, seq):
        if PIDS.get(pid) is None:
            print(f"Unknown PID: {pid}")
            return
        
        value = PIDS[pid].func(data0, data1)

        with self.lock:
            # print(f"Processing PID: {pid}, Value: {value} {PIDS[pid].unit}, Seq: {seq}")
            # for metric in computedMetrics.values():
            if pid == "0x0D":
                rpm_data = self.mostRecentValues.get("0x0C")
                if rpm_data:
                    self.currentGear = self.estimate_gear(value, rpm_data[0])
                #if(self.currentSpeed is not None):
                time = seq * PIDS[pid].period_ms / 1000.0
                speed_ms = value / 3.6

                self.speedWindow.append(speed_ms)
                    # self.timeWindow.append(time)
                dydt = applySGFilter(self.speedWindow, pid)
                
                self.currentAcc = dydt
                # self.currentSpeed = (value, seq)
            if pid == "0x0C":
                speed_data = self.mostRecentValues.get("0x0D")
                if speed_data:
                    self.currentGear = self.estimate_gear(speed_data[0], value)
                # self.currentGear = self.estimate_gear(self.mostRecentValues.get("0x0D")[0], value)

            # print(f"PID: {pid}, Value: {value} {PIDS[pid].unit}, Seq: {seq}")
    
            if pid == "0x10": # derive inst fuel cons
                speed, speedSeq = self.mostRecentValues.get("0x0D")
                speedTime = speedSeq * PIDS["0x0D"].period_ms / 1000.0
                mafTime = seq * PIDS[pid].period_ms / 1000.0
                if abs(speedTime - mafTime) < 0.5 and speed != 0: # compare timestamps
                    ff = (value * 3600) / (14.5 * 720)
                    fCons = ff / speed
                    self.fuelCons = fCons
                    # print(f"Instant fuel cons: {fCons:.2f} l/km")

            self.pidValues[pid].append((value, seq))
            self.mostRecentValues[pid] = (value, seq)



    def get_most_recent(self):
        with self.lock:
            latestData = {
                pid: {"value": value, "unit": PIDS[pid].unit}
                for pid, (value, seq) in self.mostRecentValues.items()
            }
            return {
                "latestData": latestData, 
                "accel": self.currentAcc,
                "fuelCons": self.fuelCons,  
                "gear": self.currentGear
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
    notConnected = True
    print("Attempting to connect to serial port...")

    while notConnected:
        port = find_connected_port()
        # port = "COM4"
        if port is None:
            print("No serial port found. Retrying in 1 second...")
            sleep(1)
        else:
            ser = serial.Serial(port, 115200)
            print(f"Connected to serial port: {port}")
            notConnected = False
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

    with open('data_log2.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            pid = row['PID']
            data0 = int(row['Data0'], 16)
            data1 = int(row['Data1'], 16)
            seq = int(row['Seq'])
            store.process_packet(pid, data0, data1, seq)
            sleep(delay)



# pidValues = defaultdict(list)
# mostRecentValues = {}

# #previousSpeed = None # [(val, seq)]
# currentSpeed = None # (value, seq)
# currentAcc = None

# # WINDOWSIZE = 9
# # speed_window = deque(maxlen=WINDOWSIZE)

# times = []
# speeds = []
# accs = []

# with open('data_log2.csv', 'r') as csvfile:
#     reader = csv.DictReader(csvfile)
#     for row in reader:
#         pid = row['PID']
#         data0 = int(row['Data0'], 16)
#         data1 = int(row['Data1'], 16)
#         seq = int(row['Seq'])
#         value = pids[pid].func(data0, data1)
#         if(pid == "0x0D"):

#             ## needs fixing
#             # time_ms = seq * pids[pid].period + (pids[pid].period/2)
#             # speed_ms = value / 3.6
#             # speed_window.append((speed_ms, time_ms))
#             # if(len(speed_window) < WINDOWSIZE):
#             #     continue # not enough data to calc acc
#             # times = np.array([t for _, t in speed_window])
#             # speeds = np.array([s for s, _ in speed_window])

#             # local_t = times - times[-1] # offset times to 0 at the most recent sample
#             # coefficients = np.polyfit(local_t, speeds, 2)
#             # currentAcc = coefficients[1] # we are at t=0 so acc is just the linear term
#             # print(f"Speed: {value} km/h, Acc: {currentAcc:.2f} m/s^2 at time {(seq * pids[pid].period)/1000} s")

#             if(currentSpeed is not None):
#                 prevVal, prevSeq = currentSpeed
#                 dt = (seq - prevSeq) * pids[pid].period / 1000.0 # convert to seconds

#                 if(dt > 0):
#                     dv = (value - prevVal) / 3.6
#                     currentAcc = dv / dt
#                     accs.append((currentAcc, seq))
#                     print(f"Speed: {value} km/h, Acc: {currentAcc:.2f} m/s^2 at time {(seq * pids[pid].period)/1000} s")
#             #     # convert to m/s, then div by number time between samples -> diff * period then conv to s
#             currentSpeed = (value, seq)
#             # speeds.append((value, seq))
#             #times.append(seq * pids[pid].period / 1000.0)
#         pidValues[pid].append((value, seq))
#         mostRecentValues[pid] = (value, seq)
#         # print(f"PID: {pid}, Value: {value} {pids[pid].unit}, Seq: {seq}")
#         # pidValues[pid].append((data0, data1, seq))
#         sleep(1/128) # average of 16 samples/s

    #print(f"PID: {pid}, Data: {data0:02X} {data1:02X}, Seq: {seq}")


# fig, ax1 = plt.subplots(figsize=(10, 6))
# timesy = []
# speedsy = []

# for a, seq in speeds:
#     print(f"Speed: {a} km/h at time {seq * 333} ms")
#     x = seq * 333 / 1000  # Convert ms to seconds
#     y = a
#     timesy.append(x)
#     speedsy.append(y)

# ax1.plot(timesy, speedsy, color="tab:blue", linewidth=1.5, label="Speed(km/h)")
# ax1.set_xlabel("Time (s)")
# ax1.set_ylabel("Speed (km/h)", color="tab:blue")
# ax1.tick_params(axis="y", labelcolor="tab:blue")

# timesx = []
# thtrrlx = []
# maxRPM = 0

# for a, seq in accs:
#   #  print(f"RPM: { (a * 256 + b )/ 4 } at time {seq * 333} ms")
#     x = seq * 333 / 1000  # Convert ms to seconds
#     timesx.append(x)
#     thtrrlx.append(a)

# # print(f"Max RPM: {maxRPM}")

# ax2 = ax1.twinx()
# ax2.plot(timesx, thtrrlx, color="tab:red", linewidth=1, label="RPM")
# ax2.set_ylabel("RPM", color="tab:red")
# ax2.tick_params(axis="y", labelcolor="tab:red")

# fig.tight_layout()
# plt.show()