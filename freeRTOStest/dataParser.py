# test script to analyse the data log from recieve.py

import csv
import threading
from collections import defaultdict, deque
import matplotlib.pyplot as plt
from dataclasses import dataclass
#from scipy.signal import savgol_filter
import numpy as np
from time import sleep


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


class Analyser:
    def __init__(self):
        self.lock = threading.Lock()
        self.pidValues = defaultdict(list)
        self.mostRecentValues = {}
        self.currentSpeed = None
        self.currentAcc = None

    def process_packet(self, pid, data0, data1, seq):
        if PIDS.get(pid) is None:
            print(f"Unknown PID: {pid}")
            return
        
        value = PIDS[pid].func(data0, data1)

        with self.lock:
            # print(f"Processing PID: {pid}, Value: {value} {PIDS[pid].unit}, Seq: {seq}")
            if pid == "0x0D":
                if(self.currentSpeed is not None):
                    # print(f"Previous speed: {self.currentSpeed[0]} km/h at seq {self.currentSpeed[1]}")
                    prevVal, prevSeq = self.currentSpeed
                    dt = (seq - prevSeq) * PIDS[pid].period_ms / 1000.0 # convert to seconds
                    # print(f"Time since last speed sample: {dt:.3f} s")
                    if(dt > 0):
                        dv = (value - prevVal) / 3.6
                        self.currentAcc = dv / dt
                        #self.accs.append((self.currentAcc, seq))
                        # print(f"Speed: {value} km/h, Acc: {self.currentAcc:.2f} m/s^2 at time {(seq * PIDS[pid].period_ms)/1000} s")
                self.currentSpeed = (value, seq)

            # print(f"PID: {pid}, Value: {value} {PIDS[pid].unit}, Seq: {seq}")
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
                "accel": self.currentAcc
            }

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