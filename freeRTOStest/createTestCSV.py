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
        values = {7:(0x07,0xD, 0xFF, 0x42), # low confid incorr
                1:(0x15,0x02, 0x09, 0x85),
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
        key = 0
        # Write rows
        for num in range(num_rows + 1):
            if num % 50 == 0:
                key = (key % 7) + 1
            rpm0, rpm1, speed, throttle = values[key]
            writer.writerow(["0x0C", f"{rpm0:02X}", f"{rpm1:02X}", RPMseq])
            writer.writerow(["0x0C", f"{rpm0:02X}", f"{rpm1:02X}", RPMseq+1])
            writer.writerow(["0x0C", f"{rpm0:02X}", f"{rpm1:02X}", RPMseq+2])
            RPMseq += 3
            writer.writerow(["0x0D", f"{speed:02X}", "00", speedSeq])
            writer.writerow(["0x0D", f"{speed:02X}", "00", speedSeq+1])
            writer.writerow(["0x0D", f"{speed:02X}", "00", speedSeq+2])
            speedSeq += 3

            writer.writerow(["0x11", f"{throttle:02X}", "00", throttleSeq])
            throttleSeq += 1


    print(f"CSV file '{filename}' created with {num_rows} rows.")

# Call the function to create the CSV file
create_csv("gearEstimationTest.csv", 350)