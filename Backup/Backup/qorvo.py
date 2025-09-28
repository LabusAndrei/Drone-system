import serial
import time
import datetime

DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Conectat pe " +DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
while True:
    try:
        line=DWM.readline()
        if(line):
            if len(line)>=140:
                parse=line.decode().split(",")
                x_pos=parse[parse.index("POS")+1]
                y_pos=parse[parse.index("POS")+2]
                z_pos=parse[parse.index("POS")+3]
                val = (x_pos,y_pos,z_pos)
                print("Time:", datetime.datetime.now().strftime("%H:%M:%S"),"X:",x_pos,",Y:",y_pos,",Z:",z_pos)
            else:
                #print("Pozitie necalculata: ",line.decode())
                time.sleep(0.01)
                
    except Exception as ex:
        print(ex)
        break
DWM.write("\r".encode())
DWM.close()
