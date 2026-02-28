import serial
import struct

ser = serial.Serial('COM4', 115200)
ser.flushInput()
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
            except ValueError:
                print(f"Invalid data format")
    else:
        print("No data received")