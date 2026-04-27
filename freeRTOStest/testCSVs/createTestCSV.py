import csv

def create_csv(filename, num_rows):
    # Open the file in write mode
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write the header
        writer.writerow(["PID", "Data0", "Data1", "Seq"])
        
        # rpm, speed, throttle, gear
        # 1654.5, 83.0,28.627450980392158,6
        # 1499.0, 66.0,69.80392156862744,5
        # 2049.0, 71,  27.84313725490196,4
        # 1732.75,43,  25.88235294117647,3
        # 1984.25,29,72.54901960784314,2
        # 1344.5, 9,   52.15686274509804,1
        values = {1:(0x15,0x02, 0x09, 0x85),
                  2:(0x1F,0x01, 0x1D, 0xB9),
                  3:(0x1B,0x13, 0x2B, 0x42),
                  4:(0x20,0x04, 0x47, 0x47),
                  5:(0x17,0x6C, 0x42, 0xB2),
                  6:(0x19,0xDA, 0x53, 0x49),
                  } 

        # Initialize values
        pid = "0x05"
        data0 = 0x41
        data1 = 0x00
        RPMseq = 0
        speedSeq = 0
        throttleSeq = 0
        tempSeq = 0
        loadSeq = 0
        key = 0

        speedVal = 0x05
        rpmVal = (0x1F, 0x40) # rpm = 2000
        # Write rows
        for num in range(num_rows + 1):
            # if num % 50 == 0:
            if num < 100: # idle
                # rpm0, rpm1, speed, throttle = values[key]
                writer.writerow(["0x0C", f"{0x0D:02X}", f"{0x48:02X}", RPMseq])
                writer.writerow(["0x0C", f"{0x0D:02X}", f"{0x48:02X}", RPMseq+1])
                writer.writerow(["0x0C", f"{0x0D:02X}", f"{0x48:02X}", RPMseq+2])
                RPMseq += 3
                # zero speed
                writer.writerow(["0x0D", f"{0x00:02X}", "00", speedSeq])
                writer.writerow(["0x0D", f"{0x00:02X}", "00", speedSeq+1])
                writer.writerow(["0x0D", f"{0x00:02X}", "00", speedSeq+2])
                speedSeq += 3

                writer.writerow(["0x05", f"{0x87:02X}", "00", tempSeq])
                writer.writerow(["0x05", f"{0x87:02X}", "00", tempSeq+1])
                tempSeq += 2

                writer.writerow(["0x04", f"{0x00:02X}", "00", loadSeq])
                writer.writerow(["0x04", f"{0x00:02X}", "00", loadSeq+1])
                loadSeq += 2


                writer.writerow(["0x11", f"{0x40:02X}", "00", throttleSeq])
                writer.writerow(["0x11", f"{0x40:02X}", "00", throttleSeq + 1])
                throttleSeq += 2

                
            elif num < 200: # accelerating
                if True:
                    if speedVal < 0x70:
                        speedVal += 2 # acc at 1 km/h/s
                    raw = (rpmVal[0] << 8) | rpmVal[1]
                    if raw < 1000:
                        raw += 100
                        rpmVal = ((raw >> 8 )& 0xFF, raw & 0xFF)
                
                writer.writerow(["0x0D", f"{speedVal}", "00", speedSeq])
                writer.writerow(["0x0D", f"{speedVal}", "00", speedSeq+1])
                writer.writerow(["0x0D", f"{speedVal}", "00", speedSeq+2])
                speedSeq += 3

                writer.writerow(["0x0C", f"{rpmVal[0]:02X}", f"{rpmVal[1]:02X}", RPMseq])
                writer.writerow(["0x0C", f"{rpmVal[0]:02X}", f"{rpmVal[1]:02X}", RPMseq+1])
                writer.writerow(["0x0C", f"{rpmVal[0]:02X}", f"{rpmVal[1]:02X}", RPMseq+2])
                RPMseq += 3

                writer.writerow(["0x05", f"{0x87:02X}", "00", tempSeq])
                writer.writerow(["0x05", f"{0x87:02X}", "00", tempSeq+1])
                tempSeq += 2

                writer.writerow(["0x04", f"{0xBF:02X}", "00", loadSeq])
                writer.writerow(["0x04", f"{0xBF:02X}", "00", loadSeq+1])
                loadSeq += 2

                writer.writerow(["0x11", f"{0xBF:02X}", "00", throttleSeq])
                writer.writerow(["0x11", f"{0xBF:02X}", "00", throttleSeq + 1])
                throttleSeq += 2
            

    print(f"CSV file '{filename}' created with {num_rows} rows.")

# Call the function to create the CSV file
create_csv("accAndHighLoad.csv", 350)