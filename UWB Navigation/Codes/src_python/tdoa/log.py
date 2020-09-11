import serial
import numpy as np
import threading
import time

rate = "10Hz_5ms_new"


ser1 = serial.Serial("/dev/ttyUSB0", 576000)
ser2 = serial.Serial("/dev/ttyUSB1", 576000)

numFrames = 100


''' [ [frameId, timestamp] ... ] '''

anc1_dat = []
anc2_dat = []

anc1Done = 0
anc2Done = 0

count1 = 0
count2 = 0

def readAnc1():
    global anc1Done
    global anc1_dat
    global count1
    for i in range(numFrames):
        lin = ser1.readline().decode("ascii")[:-2].split(",")
        anc1_dat.append( [ int(lin[1]), int(lin[2]) ])
        count1 += 1
    anc1Done = 1
    

def readAnc2():
    global anc2Done
    global anc2_dat
    global count2
    for i in range(numFrames):
        lin = ser2.readline().decode("ascii")[:-2].split(",")
        anc2_dat.append( [ int(lin[1]), int(lin[2]) ])
        count2 += 1
    anc2Done = 1



t1 = threading.Thread(target=readAnc1) 
t2 = threading.Thread(target=readAnc2) 
t1.start()
t2.start()

while(True):
    time.sleep(0.01)
    print ((count1+count2)/2, end="\r")
    if (anc1Done == 1 and anc2Done ==1):
        print("Completed, writing to file")
        with open(rate+"_a12_diff.csv", "w") as f:
            for d in anc1_dat:
                for e in anc2_dat:
                    if d[0] == e[0] :
                        if(abs(d[1]-e[1]) < 10000000):
                            f.write(str(d[0]) + "," + str(15.65*10e-3*(d[1]-e[1])))
                            f.write("\n")
        exit(0)




