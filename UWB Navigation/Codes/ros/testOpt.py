import pickle
import matplotlib
import matplotlib.pyplot as plt
from scipy.optimize import optimize
import numpy as np



ac = np.zeros((4, 3))
ac[0] = np.array([-2.68, 1.8, 1.81])  # x, height, y
ac[1] = np.array([-2.7, 1.63, -1.5])
ac[2] = np.array([2.54, 1.81, -1.3])
ac[3] = np.array([2.42, 1.58, 1.62])



#ac = np.zeros((4, 2))
#ac[0] = np.array([-2.68, 1.81])  # x, height, y
#ac[1] = np.array([-2.7, -1.5])
#ac[2] = np.array([2.54, -1.3])
#ac[3] = np.array([2.42, 1.62])



def getPos(p1, p2, p3, r1, r2, r3):

    A = -2*p1[0] + 2*p2[0]
    B = -2*p1[2] + 2*p2[2]
    C = r1**2 - r2**2 - p1[0]**2 + p2[0]**2 - p1[2]**2 + p2[2]**2

    D = -2*p2[0] + 2*p3[0]
    E = -2*p2[2] + 2*p3[1]
    F = r2**2 - r3**2 - p2[0]**2 + p3[0]**2 - p2[2]**2 + p3[1]**2

    pos = [round((C*E - F*B)/(E*A - B*D), 2),
           round((C*D - A*F)/(B*D - A*E), 2)]
    return pos





def f_opt(pos, radii):
    dist = 0.
    for i in range(0, 4):
        dist += np.square(np.linalg.norm(pos-ac[i]) - radii[i])
    return dist


def main():

    offsets = np.array([-0.66171298, -0.76921925, -0.70283083, -0.91615231])

    with open("uwb.pkl","r") as f:
        data = pickle.load(f)

    arr = np.array(data)

    
    pos_opt = np.array([0.,0.,0.])
    pos_opt1 = np.array([0.,0.,0.])
    pos = np.array([0.,0.])
    pts_opt = []
    pts_opt1 = []
    pts = []

    arr = arr[100:]

    for row in arr:
        pos_opt1 = optimize.fmin_powell(f_opt, pos_opt1, args=(
            row[0:4]+offsets,), xtol=0.0001, ftol=0.0001, disp=0)

        pos_opt = optimize.fmin_bfgs(f_opt, pos_opt, args=(row[0:4]+offsets,), disp=0)
        pos = getPos(ac[0], ac[1], ac[2], row[0], row[1], row[2])


        pts_opt1.append(pos_opt1)
        pts_opt.append(pos_opt)
        pts.append(pos)
        #print(np.linalg.norm(pos - row[4:7]))
    
    arr = np.array(arr)
    pts = np.array(pts)
    pts_opt = np.array(pts_opt)
    pts_opt1 = np.array(pts_opt1)

    plt.figure()
    plt.scatter(pts_opt[:,0], pts_opt[:,2], c="g")
    #plt.scatter(pts_opt1[:,0], pts_opt1[:,2], c="k")
    plt.scatter(pts[:,0], pts[:,1], c="b")
    plt.scatter(arr[:,4], arr[:,6], c="r")

    plt.show()


#    pts = np.array(pts)
#    plt.figure()
#    plt.scatter(pts[:,0], pts[:,1])
#    plt.show()


if __name__ == "__main__":
    main()


