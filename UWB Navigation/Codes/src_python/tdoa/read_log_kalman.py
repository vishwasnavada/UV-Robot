import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from scipy.stats import norm
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter as KF


#a_fl_name = "./audi_1_2_m/audi_1_2_m_0_a.csv"
#b_fl_name = "./audi_1_2_m/audi_1_2_m_0_b.csv"
#a_fl_name = "./audi_0.5_4_m/audi_0.5_4_m_0_a.csv"
#b_fl_name = "./audi_0.5_4_m/audi_0.5_4_m_0_b.csv"
a_fl_name = "./audi_1_1_m/audi_1_1_m_1_a.csv"
b_fl_name = "./audi_1_1_m/audi_1_1_m_1_b.csv"
#a_fl_name = "./audi_0_2_m/audi_0_2_m_0_a.csv"
#b_fl_name = "./audi_0_2_m/audi_0_2_m_0_b.csv"
#a_fl_name = "./audi_0_1_m/audi_0_1_m_1_a.csv"
#b_fl_name = "./audi_0_1_m/audi_0_1_m_1_b.csv"
#a_fl_name = "./audi_1_2_m/audi_1_2_m_0_a.csv"
#b_fl_name = "./audi_1_2_m/audi_1_2_m_0_b.csv"

a = []
b = []

pred = []



wrapStart = 0.

class UWBKF(KF):
    def __init__(self):
        self.dt_uwb = 0.01
        KF.__init__(self, 2, 2)
        self.syncWrapMult = 0
        self.ancWrapMult = 0
        self.transmitWrapMult = 0
        self.wrapVal = 1099511627776
        self.prevTransmitTime = 0
        self.measPrev  = []


    def predict(self, u):
        delta = u[1] - u[0]
        F = np.array([ [1, delta], [0, 1] ])
        KF.predict(self, F=F)


    def getMeas(self, ta, ts):
        t = ta[1]
        m = (ta[1] - ta[0]) / (ts[1] - ts[0])
        if( m == 0 ):
            m = self.measPrev[1]
        self.measPrev = [t,m]
        return np.array([t, m])

    def transmitWrap(self, timeNow) :
        timeNow += self.transmitWrapMult*self.wrapVal
        if( timeNow - self.prevTransmitTime < 0):
            self.transmitWrapMult += 1
            timeNow += self.wrapVal
        self.prevTransmitTime = timeNow
        return timeNow

    def syncWrap(self, ts):
        ts = ts + np.repeat(self.syncWrapMult*self.wrapVal,2)
        if( ts[1] - ts[0] < 0):
            self.syncWrapMult +=  1
            ts[1] +=  self.wrapVal
        return ts

    def anchorWrap(self, ta):
        ta = ta + np.repeat(self.ancWrapMult*self.wrapVal,2)
        if( ta[1] - ta[0] < 0):
            self.ancWrapMult +=  1
            ta[1] +=  self.wrapVal
        return ta



flag = 0



ka = UWBKF()
kb = UWBKF()





testa = []
testb = []

with open(a_fl_name, "r") as f:
    dat = f.readlines()
    for d in dat:
        a.append([ int(x) for x in d[:-1].split(",")[:-1]])
        testa.append(float(d[:-1].split(",")[-1]))

with open(b_fl_name, "r") as f:
    dat = f.readlines()
    for d in dat:
        b.append([ int(x) for x in d[:-1].split(",")[:-1]])
        testb.append(float(d[:-1].split(",")[-1]))





a = np.array(a)
b = np.array(b)


'''
ta_s1, ta_s2, ta_tx, ts1, ts2, tx_tx_adjusted
'''

idx = 0

std_T = 50
std_m = 0.5

std_meas_T = 1000
std_meas_m = 2

''' Load the initial parameters '''
ma = (a[idx,1] - a[idx,0])/(a[idx,4] - a[idx,3])
ta = a[idx,1]

ka.x = np.array([ta, ma])
ka.P = np.diag([std_T**2, std_m**2])
ka.R = np.diag([std_meas_T**2, std_meas_m**2])
ka.H = np.diag([1,1])

''' Load the initial parameters '''
mb = (b[idx,1] - b[idx,0])/(b[idx,4] - b[idx,3])
tb = b[idx,1]
kb.x = np.array([tb, mb])
kb.P = np.diag([std_T**2, std_m**2])
kb.R = np.diag([std_meas_T**2, std_meas_m**2])
kb.H = np.diag([1,1])

print("Initial a", ka.x)
input()


tas = []
tbs = []
zas = []
zbs = []
tta = []
ttb = []
mtta = []
mttb = []
tss = []

i = 0 
for row in a[idx+1:,:]:
    ts = ka.syncWrap(np.array([row[3], row[4]]))
    tss.append(ts[1])
    ka.predict(ts)
    z = ka.getMeas(ka.anchorWrap(np.array([row[0], row[1]])), ts)
    zas.append(z[0])
    tas.append(ka.x[0])
    ka.update(z)
    mtta.append((ka.transmitWrap(row[2])-z[0])/z[1])
    tta.append((ka.transmitWrap(row[2])-ka.x[0])/ka.x[1])


for row in b[idx+1:,:]:
    ts = kb.syncWrap(np.array([row[3], row[4]]))
    kb.predict(ts)
    z = kb.getMeas(kb.anchorWrap(np.array([row[0], row[1]])), ts)
    zbs.append(z[0])
    tbs.append(kb.x[0])
    kb.update(z)
    mttb.append((kb.transmitWrap(row[2])-z[0])/z[1])
    ttb.append((kb.transmitWrap(row[2])-kb.x[0])/kb.x[1])


tas = np.array(tas)
tbs = np.array(tbs)

zas = np.array(zas)
zbs = np.array(zbs)

tta = np.array(tta)
ttb = np.array(ttb)

mtta = np.array(mtta)
mttb = np.array(mttb)


tdiff = tas-tbs
mdiff = zas-zbs


diff = 15.65e-3*(ttb-tta)
print(np.mean(diff), np.var(diff))
zdiff = 15.65e-3*(mttb-mtta)



mu, sigma = norm.fit(diff)
n, bins, patches = plt.hist(diff, density=True, label="Sample bins")
y = mlab.normpdf( bins, mu, sigma)
l = plt.plot(bins, y, 'r--', linewidth=2, label="Estimated distribution")
plt.grid(True)
plt.title(r"$\mathcal{N}("+str(round(mu,3))+"ns,"+str(round(sigma,3))+"ns^2$)")
plt.legend()
plt.xlabel("Time drift between A1 and A2 in $n$s")
plt.ylabel("Probability")
#plt.scatter(tss,diff)
#plt.scatter(tss,zas-tas)
#plt.scatter(tss,tdiff)
#plt.scatter(tss,mdiff)

#plt.scatter(tss,zbs)
#plt.scatter(tss,tas)
#plt.scatter(tss,tbs)

#plt.scatter(tss,zdiff-adiff)

#diffm = 15.65e-3*(mttb-mtta)
#mtta = np.array(mtta)
#mttb = np.array(mttb)
#plt.scatter(tss,diffm)

#plt.scatter(tss,tas)
#plt.scatter(tss,tbs)

plt.show()
