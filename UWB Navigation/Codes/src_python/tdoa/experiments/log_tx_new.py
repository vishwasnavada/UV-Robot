import serial
import numpy as np
import threading
import time


#name = "p_5.2_3_m"
#f_a = "./p_5.2_3/anchor1_5.2_3.log"
#f_b = "./p_5.2_3/anchor2_5.2_3.log"
#f_c = "./p_5.2_3/anchor3_5.2_3.log"
#f_s = "./p_5.2_3/synca_5.2_3.log"
#outdir = "./p_5.2_3/"


name = "p_0_2_m"
f_a = "./p_0_2_m/anchor1_0_2.log"
f_b = "./p_0_2_m/anchor2_0_2.log"
f_c = "./p_0_2_m/anchor3_0_2.log"
outdir = "./p_0_2_m/"
f_s = "./p_0_2_m/synca_0_2.log"



''' [ [frameId, timestamp] ... ] '''

anc1_dat = []
anc1_sync = []
anc2_dat = []
anc2_sync = []
anc3_dat = []
anc3_sync = []

sync_dat = []

anc1Done = 0
anc2Done = 0
anc3Done = 0

syncDone = 0

count1 = 0
count2 = 0
count3 = 0

syncCount = 0



def readAnc1():
    global count1
    global anc1_dat
    global anc1Done
    with open(f_a, "r") as ser1:
        dat1 = ser1.readlines()
        for line in dat1:
            lin = line[:-1].split(",")
            ''' If sync '''
            if (int(lin[0]) == 2):
                anc1_sync.append( [ int(lin[2]), int(lin[3]) ])
            ''' If range '''
            if (int(lin[0]) == 3):
                anc1_dat.append( [ int(lin[2]), int(lin[3]) ])
            count1 += 1
    anc1Done = 1
    

def readAnc2():
    global count2
    global anc2_dat
    global anc2Done
    with open(f_b, "r") as ser2:
        dat2 = ser2.readlines()
        for line in dat2:
            lin = line[:-1].split(",")
            ''' If sync '''
            if (int(lin[0]) == 2):
                anc2_sync.append( [ int(lin[2]), int(lin[3]) ])
            ''' If range '''
            if (int(lin[0]) == 3):
                anc2_dat.append( [ int(lin[2]), int(lin[3]) ])
            count2 += 1
    anc2Done = 1

def readAnc3():
    global count3
    global anc3_dat
    global anc3Done
    with open(f_c, "r") as ser3:
        dat3 = ser3.readlines()
        for line in dat3:
            lin = line[:-1].split(",")
            ''' If sync '''
            if (int(lin[0]) == 2):
                anc3_sync.append( [ int(lin[2]), int(lin[3]) ])
            ''' If range '''
            if (int(lin[0]) == 3):
                anc3_dat.append( [ int(lin[2]), int(lin[3]) ])
            count3 += 1
    anc3Done = 1


def readSync():
    global syncDone
    global sync_dat
    global syncCount
    with open(f_s, "r") as ser4:
        sy  = ser4.readlines()
        for line in sy:
            lin = line[:-1].split(",")
            if(lin[0] == "2"):
                sync_dat.append( [ int(lin[1]), int(lin[2]) ])
                syncCount += 1
    syncDone = 1



def wrap(timeNow) :
    if(timeNow < 0):
        timeNow += 1099511627776
    return timeNow



readAnc1()
readAnc2()
readAnc3()
readSync()

anc1_dat = np.array(anc1_dat)
anc2_dat = np.array(anc2_dat)
anc3_dat = np.array(anc3_dat)
anc1_sync = np.array(anc1_sync)
anc2_sync = np.array(anc2_sync)
anc3_sync = np.array(anc3_sync)
sync_dat = np.array(sync_dat)



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

missed = 0


numFrames = 800

while(True):
    time.sleep(0.01)
    if (anc1Done == 1 and anc2Done ==1 and anc3Done == 1 and syncDone == 1):
        print("Completed, writing to file")
        with open(outdir+name+"_c.csv", "w") as e, open(outdir+name+"_a.csv", "w") as f, open(outdir+name+"_b.csv","w") as g, open(outdir+name+"_diff.csv","w") as h:
            ts_prev = sync_dat[0][1]
            ta_prev = anc1_dat[0][1]
            tb_prev = anc2_dat[0][1]
            tc_prev = anc3_dat[0][1]
            for s in range(1,numFrames):
                try:

                    ts2 = sync_dat[np.where(sync_dat[:,0] == s)][-1][1]
                    ts1 = ts_prev

                    ta_tx = anc1_dat[np.where(anc1_dat[:,0] == s-1)][-1][1]
                    tb_tx = anc2_dat[np.where(anc2_dat[:,0] == s-1)][-1][1]
                    tc_tx = anc3_dat[np.where(anc3_dat[:,0] == s-1)][-1][1]


                    ta_s2 = anc1_sync[np.where(anc1_sync[:,0] == s)][-1][1] 
                    ta_s1 = ta_prev 

                    tb_s2 = anc2_sync[np.where(anc2_sync[:,0] == s)][-1][1] 
                    tb_s1 = tb_prev 

                    tc_s2 = anc3_sync[np.where(anc3_sync[:,0] == s)][-1][1] 
                    tc_s1 = tc_prev 


                    f.write(str(s)+ ","+str(ta_s1)+","+ str(ta_s2)+","+ str(ta_tx)+","+ str(ts1)+","+ str(ts2)+"\n")
                    g.write(str(s)+","+str(tb_s1)+","+ str(tb_s2)+","+ str(tb_tx)+","+ str(ts1)+","+ str(ts2)+"\n")
                    e.write(str(s)+","+str(tc_s1)+","+ str(tc_s2)+","+ str(tc_tx)+","+ str(ts1)+","+ str(ts2)+"\n")
                    ts_prev = ts2
                    ta_prev = ta_s2
                    tb_prev = tb_s2
                    tc_prev = tc_s2
                except Exception as x:
                    missed += 1
            print("Missed", missed)
            exit()

