import serial
import struct
import time
from queue import Queue

def raw_to_speed(dataA, dataB):
    return dataA

def raw_to_maf(dataA, dataB):
# ((replyFrame[pos] << 8) + replyFrame[pos + 1]) / 100.0f;
    return ((dataA << 8) + dataB) / 100.0    

pidToUnit = {
    "0x0C": "rpm",
    "0x42": "v",
    "0x23": "kPa",
    "0x0D": "km/h",
    "0x10": "g/s",
    "0x1F": "s",
    "0x11": "%",
    "0x0F": "°C",
    "0x05": "°C",
    "0x04": "%"
}

#class dataPoint:
notConnected = True
print("Attempting to connect to serial port...")
while notConnected:
    try:
        ser = serial.Serial('COM4', 115200)
        print("Connected to serial port.")
        notConnected = False
    except serial.SerialException:
        time.sleep(1)
ser.flushInput()

mafQ = Queue(10)
speedQ = Queue(10)
while True:
    line = ser.readline()
    #print(f"Raw line: {line}")
    if line:
        line = line.decode("utf-8").strip()
        parts = line.split(',')
        if (len(parts) == 4):
            pid = parts[0]
            try: 
                data0 = int(parts[1], 16)
                data1 = int(parts[2], 16) 
                seq = int(parts[3].strip(),10)
                print(f"PID: {pid}, Data: {data0:02X} {data1:02X}, Seq: {seq}")
                with open ("data_log2.txt", "a") as f:
                    f.write(f"{pid},{data0:02X},{data1:02X},{seq}\n")
                # if pid == "0x0D":
                #     if(speedQ.full()):
                #         speedQ.get()  # Remove the oldest item if the queue is full
                #     speedQ.put((seq, raw_to_speed(data0, data1)))
                # if pid == "0x10":
                #     if(mafQ.full()):
                #         mafQ.get() 
                #     mafQ.put((seq, raw_to_maf(data0, data1)))
                # if(mafQ.qsize() > 0 and speedQ.qsize() > 0):
                #     mafSeq, mafVal = mafQ.get()
                #     speedSeq, speedVal = speedQ.get()
                #     if(speedVal != 0):
                #         ff = (mafVal * 3600) / (14.5 * 720)
                #         fCons = ff / speedVal
                #         print(f"Instant fuel cons: {fCons:.2f} L")
            except ValueError:
                print(f"Invalid data format")
    else:
        print("No data received")