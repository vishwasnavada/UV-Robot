import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from scipy.stats import norm


fl_name = "./old/new_audi_50ms_0_1_m_diff.csv"


d = []
with open(fl_name, "r") as f:
    dls = f.readlines()
    for ds in dls:
        d.append(float(ds))

d = np.array(d)
mu, sigma = norm.fit(d)
n, bins, patches = plt.hist(d, density=True, label="Sample bins")
y = mlab.normpdf( bins, mu, sigma)
l = plt.plot(bins, y, 'r--', linewidth=2, label="Estimated distribution")
plt.grid(True)
plt.title(r"$\mathcal{N}("+str(round(mu,3))+"ns,"+str(round(sigma,3))+"ns^2$)")
plt.legend()
plt.xlabel("Time drift between A1 and A2 in $n$s")
plt.ylabel("Probability")
plt.show()
