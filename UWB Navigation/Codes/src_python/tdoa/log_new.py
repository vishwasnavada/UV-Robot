import serial
import numpy as np
import threading
import time

name = "test_4"


ser1 = serial.Serial("/dev/ttyUSB0", 576000)
ser2 = serial.Serial("/dev/ttyUSB1", 576000)
ser3 = serial.Serial("/dev/ttyUSB2", 576000)

numFrames = 500


''' [ [frameId, timestamp] ... ] '''

anc1_dat = []
anc1_sync = []
anc2_dat = []
anc2_sync = []

sync_dat = []

anc1Done = 0
anc2Done = 0
syncDone = 0

count1 = 0
count2 = 0
syncCount = 0

anchorAOffset = -3.3557
anchorBOffset = -3.3557


def readAnc1():
    global anc1Done
    global anc1_dat
    global count1
    for i in range(numFrames*2):
        lin = ser1.readline().decode("ascii")[:-2].split(",")
        ''' If sync '''
        if (int(lin[0]) == 2):
            anc1_sync.append( [ int(lin[2]), int(lin[3]) ])
        ''' If range '''
        if (int(lin[0]) == 3):
            anc1_dat.append( [ int(lin[2]), int(lin[3]) ])
        count1 += 1
    anc1Done = 1
    

def readAnc2():
    global anc2Done
    global anc2_dat
    global count2
    for i in range(numFrames*2):
        lin = ser2.readline().decode("ascii")[:-2].split(",")
        ''' If sync '''
        if (int(lin[0]) == 2):
            anc2_sync.append( [ int(lin[2]), int(lin[3]) ])
        ''' If range '''
        if (int(lin[0]) == 3):
            anc2_dat.append( [ int(lin[2]), int(lin[3]) ])
        count2 += 1
    anc2Done = 1


def readSync():
    global syncDone
    global sync_dat
    global syncCount
    for i in range(numFrames):
        lin = ser3.readline().decode("ascii")[:-2].split(",")
        sync_dat.append( [ int(lin[0]), int(lin[1]) ])
        syncCount += 1
    syncDone = 1



def wrap(timeNow) :
    if(timeNow < 0):
        timeNow += 1099511627776
    return timeNow



t1 = threading.Thread(target=readAnc1) 
t2 = threading.Thread(target=readAnc2) 
t3 = threading.Thread(target=readSync) 
t1.start()
t2.start()
t3.start()

''' 
    [ [framid, syncTxTime, anchor1S, anchor2S, anchor1TX, anchor2TX] ...]
    if syncTXTime = 0 => range message
'''

def adjustTxTime(t_s1, t_s2, ts1, ts2, t_tx):
    epsilon = 0.00000000001
    m = wrap(t_s2 - t_s1+epsilon)/wrap(ts2 - ts1 + epsilon)
    c = (t_s1 - m*ts1)
    t_tx_adjusted = (t_tx - c)/ m
    return t_tx_adjusted


tdoa = []

while(True):
    time.sleep(0.01)
    print ((count1+count2)/2, end="\r")
    if (anc1Done == 1 and anc2Done ==1 and syncDone == 1):
        print("Completed, writing to file")
        anc1_dat = np.array(anc1_dat)
        anc2_dat = np.array(anc2_dat)
        anc1_sync = np.array(anc1_sync)
        anc2_sync = np.array(anc2_sync)
        sync_dat = np.array(sync_dat)
        with open(name+"_a.csv", "w") as f, open(name+"_b.csv","w") as g, open(name+"_diff.csv","w") as h:
            for s in range(len(sync_dat)-1):
                try:
                    frame_id = sync_dat[s][0]
                    next_frame_id = sync_dat[s+1][0]

                    ts2 = sync_dat[s+1][1]
                    ts1 = sync_dat[s][1]

                    ta_s2 = anc1_sync[np.where(anc1_sync[:,0] == next_frame_id)][0][1] 
                    ta_s1 = anc1_sync[np.where(anc1_sync[:,0] == frame_id)][0][1] 
                    ta_tx = anc1_dat[np.where(anc1_dat[:,0] == frame_id)][0][1]

                    tb_s2 = anc2_sync[np.where(anc2_sync[:,0] == next_frame_id)][0][1] 
                    tb_s1 = anc2_sync[np.where(anc2_sync[:,0] == frame_id)][0][1] 
                    tb_tx = anc2_dat[np.where(anc2_dat[:,0] == frame_id)][0][1]


                    ta_tx_adjusted = adjustTxTime(ta_s1, ta_s2, ts1, ts2, ta_tx)
                    tb_tx_adjusted = adjustTxTime(tb_s1, tb_s2, ts1, ts2, tb_tx)

                    if ((15.65*10e-3)*abs(tb_tx_adjusted - ta_tx_adjusted)) < 100:
                        f.write(str(frame_id)+ ","+str(ta_s1)+","+ str(ta_s2)+","+ str(ta_tx)+","+ str(ts1)+","+ str(ts2)+","+str(ta_tx_adjusted)+"\n")
                        g.write(str(frame_id)+","+str(tb_s1)+","+ str(tb_s2)+","+ str(tb_tx)+","+ str(ts1)+","+ str(ts2)+","+str(tb_tx_adjusted)+"\n")
                        h.write(str((15.65*10e-3)*(tb_tx_adjusted - ta_tx_adjusted)) + "\n")
                except Exception as e:
                    print("Index not found")

            exit()

