import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from scipy.stats import norm
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter as KF
from filterpy.stats import plot_covariance_ellipse
from scipy.optimize import least_squares


''' Testing by interchanging '''
#a_fl_name  = "./p_0_2_m/p_0_2_m_a.csv"
#b_fl_name  = "./p_0_2_m/p_0_2_m_b.csv"
#c_fl_name  = "./p_0_2_m/p_0_2_m_c.csv"


a_fl_name  = "./p_5.2_3/p_5.2_3_m_a.csv"
b_fl_name  = "./p_5.2_3/p_5.2_3_m_b.csv"
c_fl_name  = "./p_5.2_3/p_5.2_3_m_c.csv"

posAncA = np.array([5.2, 4.3])
posAncB = np.array([0, 0])
posAncC = np.array([0, 4.3])
posSync = np.array([2, 0])


speed_light = 299792458




a = []
b = []
c = []

testa = []
testb = []
testc = []

pred = []



wrapStart = 0.



class Loco():
    def __init__(self, anc1Pos, anc2Pos, anc3Pos):
        self.anc1Pos = anc1Pos
        self.anc2Pos = anc2Pos
        self.anc3Pos = anc3Pos
        pass

    def f_opt(self, p, ancAPos, ancBPos, ancCPos, deltaTAB, deltaTAC, deltaTBC ):
        distAB = abs((speed_light * deltaTAB * 1e-9) - np.linalg.norm(p - ancAPos) + np.linalg.norm(p - ancBPos))
        distAC = abs((speed_light * deltaTAC * 1e-9) - np.linalg.norm(p - ancAPos) + np.linalg.norm(p - ancCPos))
        distBC = abs((speed_light * deltaTBC * 1e-9) - np.linalg.norm(p - ancBPos) + np.linalg.norm(p - ancCPos))
        return  distAB + distAC + distBC

    def getPos(self, t12, t13, t23):
        p = least_squares(self.f_opt, np.array([0,0]), args = (self.anc1Pos, self.anc2Pos, self.anc3Pos, t12, t13, t23), method= "trf")
        return p

class UWBKF(KF):
    def __init__(self, posAnc, posSync, std_T, std_m):
        self.posAnc = posAnc
        self.posSync = posSync
        self.std_T = std_T
        self.std_m = std_m
        KF.__init__(self, 2, 2)
        self.syncWrapMult = 0
        self.ancWrapMult = 0
        self.transmitWrapMult = 0
        self.wrapVal = 1099511627776
        self.prevTransmitTime = 0
        self.measPrev  = []
        self.prevSync = [0., 0.]
        self.prevAnc = [0., 0.]
        dOff =  np.linalg.norm(self.posAnc-self.posSync) #* 1e9 / c
        self.delSync = (dOff*1e9/speed_light)/ 15.65e-3


    def predict(self, u):
        delta = u[1] - u[0]
        F = np.array([ [1, delta], [0, 1] ])
        Q = self.std_T**2 * np.array([ [delta**2, delta], [delta, 1]])
        KF.predict(self, F=F, Q=Q)
#        print("F", F)
#        print("delta", delta)
#        print("x", self.x)
#        input()


    def getMeas(self, ta, ts):
        t = ta[1]
        m = (ta[1] - ta[0]) / (ts[1] - ts[0])
        if( m == 0 ):
            m = self.measPrev[1]
        self.measPrev = [t,m]
        return np.array([t, m])

    def transmitWrap(self, timeNow) :
        timeNow += (self.transmitWrapMult*self.wrapVal)
        while( timeNow - self.prevTransmitTime < 0):
            self.transmitWrapMult += 1
            timeNow += self.wrapVal
        self.prevTransmitTime = timeNow
        return timeNow

    def syncWrap(self, ts):
        ts = ts + np.repeat((self.syncWrapMult*self.wrapVal),2)
        while(self.prevSync[0] > ts[0]):
            ts[0] += self.wrapVal
        while( ts[1] - ts[0] < 0):
            self.syncWrapMult +=  1
            ts[1] +=  self.wrapVal
        self.prevSync = ts
        return ts

    def anchorWrap(self, ta):
        ta = ta + np.repeat((self.ancWrapMult*self.wrapVal) - self.delSync,2)
        while(self.prevAnc[0] > ta[0]):
            ta[0] += self.wrapVal
        while( ta[1] - ta[0] < 0):
            self.ancWrapMult +=  1
            ta[1] +=  self.wrapVal
        self.prevAnc = ta
        return ta






with open(a_fl_name, "r") as f:
    dat = f.readlines()
    for d in dat:
        a.append([ int(x) for x in d[:-1].split(",")[1:]])
        testa.append(float(d[:-1].split(",")[-1]))

with open(b_fl_name, "r") as f:
    dat = f.readlines()
    for d in dat:
        b.append([ int(x) for x in d[:-1].split(",")[1:]])
        testb.append(float(d[:-1].split(",")[-1]))

with open(c_fl_name, "r") as f:
    dat = f.readlines()
    for d in dat:
        c.append([ int(x) for x in d[:-1].split(",")[1:]])
        testc.append(float(d[:-1].split(",")[-1]))




a = np.array(a)
b = np.array(b)
c = np.array(c)



'''
ta_s1, ta_s2, ta_tx, ts1, ts2, tx_tx_adjusted
'''

idx = 0
endIdx = 1000

std_T = 10
std_m = 1

std_meas_T = 1
std_meas_m = 0.01


ka = UWBKF(posAncA, posSync, std_meas_T, std_meas_m)
kb = UWBKF(posAncB, posSync, std_meas_T, std_meas_m)
kc = UWBKF(posAncC, posSync, std_meas_T, std_meas_m)

''' Load the initial parameters '''
ma = (a[idx,1] - a[idx,0])/(a[idx,4] - a[idx,3])
ta = a[idx,1]

ka.x = np.array([ta, ma])
ka.P = np.diag([std_T**2, std_m**2])
ka.R = np.diag([std_meas_T**2, std_meas_m**2])
ka.H = np.diag([1,1])
''' Discrete time weiner process, tau = [rate_T, rate_m], tau * sigma * tau.T '''

''' Load the initial parameters '''
mb = (b[idx,1] - b[idx,0])/(b[idx,4] - b[idx,3])
tb = b[idx,1]
kb.x = np.array([tb, mb])
kb.P = np.diag([std_T**2, std_m**2])
kb.R = np.diag([std_meas_T**2, std_meas_m**2])
kb.H = np.diag([1,1])

''' Load the initial parameters '''
mc = (c[idx,1] - c[idx,0])/(c[idx,4] - c[idx,3])
tc = c[idx,1]
kc.x = np.array([tc, mc])
kc.P = np.diag([std_T**2, std_m**2])
kc.R = np.diag([std_meas_T**2, std_meas_m**2])
kc.H = np.diag([1,1])

print("Initial a", ka.x)
print("Initial b", kb.x)
print("Initial c", kc.x)

input()


tas = []
tbs = []
tcs = []

zas = []
zbs = []
zcs = []

tta = []
ttb = []
ttc = []

mtta = []
mttb = []
mttc = []

''' Sync time '''
tss = []

ia = []
ib = []
ic = []

i = 0 

for row in a[idx+1:endIdx,:]:
    ts = ka.syncWrap(np.array([row[3], row[4]]))
    prevAnc = ka.x[0]
    #print("ts", ts)
    #print("x", ka.x)
    #print("prevAnc", prevAnc)
    #input()
    ia.append(prevAnc)
    tss.append(ts[1])
    ka.predict(ts)
    tza = ka.anchorWrap(np.array([row[0], row[1]])) 
    z = ka.getMeas(tza, ts)
    zas.append(z[0])
    tas.append(ka.x[0])
    ka.update(z)
    tta.append(((ka.transmitWrap(row[2])-prevAnc)/ka.x[1]) + ts[0])
    mtta.append((ka.transmitWrap(row[2])-z[0])/z[1])
    i+=1


for row in b[idx+1:endIdx,:]:
    ts = kb.syncWrap(np.array([row[3], row[4]]))
    prevAnc = kb.x[0] 
    ib.append(prevAnc)
    kb.predict(ts)
    z = kb.getMeas(kb.anchorWrap(np.array([row[0], row[1]])), ts)
    zbs.append(z[0])
    tbs.append(kb.x[0])
    kb.update(z)
    ttb.append(((kb.transmitWrap(row[2])-prevAnc)/kb.x[1]) + ts[0])
    mttb.append((kb.transmitWrap(row[2])-z[0])/z[1])
    #plot_covariance_ellipse( (kb.x[0], ts[0]), kb.P, std=6, facecolor='k', alpha=0.3)

for row in c[idx+1:endIdx,:]:
    ts = kc.syncWrap(np.array([row[3], row[4]]))
    prevAnc = kc.x[0]
    ic.append(prevAnc)
    kc.predict(ts)
    z = kc.getMeas(kc.anchorWrap(np.array([row[0], row[1]])), ts)
    zcs.append(z[0])
    tcs.append(kc.x[0])
    kc.update(z)
    ttc.append(((kc.transmitWrap(row[2])-prevAnc)/kc.x[1]) + ts[0])
    mttc.append((kc.transmitWrap(row[2])-z[0])/z[1])


tss = np.array(tss)

tas = np.array(tas)
tbs = np.array(tbs)
tcs = np.array(tcs)

zas = np.array(zas)
zbs = np.array(zbs)
zcs = np.array(zcs)

tta = np.array(tta)
ttb = np.array(ttb)
ttc = np.array(ttc)

mtta = np.array(mtta)
mttb = np.array(mttb)
mttc = np.array(mttc)


''' Todo: account for first row '''
tdiff_ab = 15.65e-3*(tta-ttb)[1:]
tdiff_bc = 15.65e-3*(ttb-ttc)[1:]
tdiff_ac = 15.65e-3*(tta-ttc)[1:]
tdiffComb = np.vstack([tdiff_ab, tdiff_ac, tdiff_bc]).T

zdiff_ab = 15.65e-3*(mtta-mttb)





diffPlot = np.vstack((tss[1:], tdiff_ac))
filterIdxs = np.where(abs(diffPlot[1,:]) < 50)[0]
diffPlot = diffPlot[:,filterIdxs]
zDiffPlot = zdiff_ab

#mu, sigma = norm.fit(diffPlot[1,:])
#n, bins, patches = plt.hist(diffPlot[1,:], density=True, label="Sample bins")
#y = mlab.normpdf( bins, mu, sigma)
#l = plt.plot(bins, y, 'r--', linewidth=2, label="Estimated distribution")
#plt.grid(True)
#plt.title(r"$\mathcal{N}("+str(round(mu,3))+"ns,"+str(round(sigma,3))+"ns^2$)")
#plt.legend()
#plt.xlabel("Time drift between A1 and A2 in $n$s")
#plt.ylabel("Probability")
#print("mu", mu)
#print("sigma", sigma)


loco = Loco(posAncA, posAncB, posAncC)

posArr = []

for row in tdiffComb:
    posArr.append(loco.getPos(row[0],row[1],row[2]).x)

posArr = np.array(posArr)
mean = np.mean(posArr, axis=0)
var = np.var(posArr, axis=0)
print(mean)
input()
np.set_printoptions(precision=4)
plt.title(r" Position P(0,2), $\mathcal{N}(" + np.array2string(mean, separator=", ") + ", " + np.array2string(var, separator=", ") + ")$" )
plt.xlabel("x")
plt.ylabel("y")
H = plt.hist2d(posArr[:,0],posArr[:,1])
plt.colorbar()
plt.show()

#
##print(posArr)
#print("Mean position", np.mean(posArr, axis=0))
#print("variance position", np.mean(posArr, axis=0))

#plt.scatter(diffPlot[0,:], diffPlot[1,:])
#plt.scatter(tss,zDiffPlot)

#plt.scatter(np.arange(len(tss)),tta, c="b")
#plt.scatter(np.arange(len(tss)),ttb, c="r")
#plt.scatter(np.arange(len(tss)),ttc, c="g")

#plt.ylim(-50,100)
#plt.scatter(np.arange(len(tss)),tdiff_ac, c="r")

#print(zbs)
#plt.scatter(np.arange(0,len(tss)),tss)
#plt.scatter(tss,zas)
#plt.scatter(diffPlot[0,:],diffPlot[1,:], marker="x")
#plt.scatter(tss,tbs, marker="+")

#plt.scatter(tss,zdiff-adiff)

#diffm = 15.65e-3*(mttb-mtta)
#mtta = np.array(mtta)
#mttb = np.array(mttb)
#plt.scatter(tss,diffm)

#plt.scatter(tss,tas)
#plt.scatter(tss,tbs)
#plt.scatter(tss,tcs)

plt.show()
