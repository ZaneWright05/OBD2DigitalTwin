import serial
import csv
import os
from time import sleep
from datetime import datetime
from serial.tools import list_ports
# from model.dataParser import Parser

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

def attempt_serial_connection():
    port = find_connected_port()
        # port = "COM4"
    if port is None:
        # print("No serial port found. Retrying in 1 second...")
        return None
    try:
        ser = serial.Serial(port, 115200)
        print(f"Connected to serial port: {port}")
        return ser
    except Exception as e:
        print(f"Failed to connect to serial port {port}: {e}")
        return None
    
def read_packet(ser: serial.Serial):
    line = ser.readline()
    if not line:
        return None

    try:
        line = line.decode("utf-8", errors="ignore").strip()
        parts = line.split(",")
        if len(parts) != 4:
            return None

        pid = parts[0]
        data0 = int(parts[1], 16)
        data1 = int(parts[2], 16)
        seq = int(parts[3].strip(), 10)
        return pid, data0, data1, seq
    except Exception:
        return None


def read_csv(path, store, sample_rate=16):
    delay = 1.0 / sample_rate
    store.connected = True
    with open(path or "data_log_4.csv", "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            if not store.connected:
                break
            pid = row["PID"]
            data0 = int(row["Data0"], 16)
            data1 = int(row["Data1"], 16)
            seq = int(row["Seq"])
            store.process_packet(pid, data0, data1, seq)
            sleep(delay)

# def read_from_com(store, ser: serial.Serial = None):
#     print("Attempting to connect to serial port...")

#     if not store.connected or ser is None:
#         return
    
#     ser.flushInput()
#     directory = "logs"
#     if not os.path.exists(directory):
#         os.makedirs(directory)
#     log_name = f"{directory}/data_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
#     with open(log_name, "a") as f:
#         f.write("PID,Data0,Data1,Seq\n")
#         f.flush()
#         while True:
#             line = ser.readline()
#             if line:
#                 line = line.decode("utf-8").strip()
#                 parts = line.split(',')
#                 if (len(parts) == 4):
#                     pid = parts[0]
#                     try: 
#                         data0 = int(parts[1], 16)
#                         data1 = int(parts[2], 16) 
#                         seq = int(parts[3].strip(),10)
#                         f.write(f"{pid},{data0:02X},{data1:02X},{seq}\n")
#                         f.flush()
#                         # print(f"PID: {pid}, Data: {data0:02X} {data1:02X}, Seq: {seq}") 
#                         store.process_packet(pid, data0, data1, seq)
#                     except ValueError:
#                         print(f"Invalid data format")
#             else:
#                 print("No data received")

# def read_csv(path, store: Parser, sample_rate=16):
#     delay = 1.0 / sample_rate
#     store.connected = True
#     with open('data_log_4.csv', 'r') as csvfile:
#         reader = csv.DictReader(csvfile)
#         for row in reader:
#             pid = row['PID']
#             data0 = int(row['Data0'], 16)
#             data1 = int(row['Data1'], 16)
#             seq = int(row['Seq'])
#             store.process_packet(pid, data0, data1, seq)
#             sleep(delay)
