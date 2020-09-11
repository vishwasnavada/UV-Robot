"""
    File : localization.py
    Description : Simple code that reads the tags radial distance from 3 anchors from UART. 
    			  Input the distance between each anchor 0, 1, 2 as r01, r02, r12.   

"""


import serial
import math
import matplotlib.pyplot as plt


ser = serial.Serial("/dev/ttyACM0", 115200)

fName = "1x1----0-1-0.csv" #File name to log into
rad = open(fName+'--raw','w')
log = open(fName, 'w')

numAnchors = 3
anchorOffsets = [-0.61, -0.591, -0.703] # To be calculated from single anchor-tag calibration



# r01 = 6
# r02 = 8.485
# r12 = 6

# Obtained from absolute measurements
anchor1Pos = (0.0, 1.0)
anchor2Pos = (0.0, 0.0)
anchor3Pos = (1.0, 0.0)

ax = [min(anchor1Pos[0], anchor2Pos[0], anchor3Pos[0])-2.0, max(anchor1Pos[0], anchor2Pos[0], anchor3Pos[0])+2.0,
      min(anchor1Pos[1], anchor2Pos[1], anchor3Pos[1])-2.0, max(anchor1Pos[1], anchor2Pos[1], anchor3Pos[1])+2.0]
plt.axis(ax)

# Assume origin is p1


def getPos(p1, p2, p3, r1, r2, r3):

    A = -2*p1[0] + 2*p2[0]
    B = -2*p1[1] + 2*p2[1]
    C = r1**2 - r2**2 - p1[0]**2 + p2[0]**2 - p1[1]**2 + p2[1]**2

    D = -2*p2[0] + 2*p3[0]
    E = -2*p2[1] + 2*p3[1]
    F = r2**2 - r3**2 - p2[0]**2 + p3[0]**2 - p2[1]**2 + p3[1]**2

    pos = ((C*E - F*B)/(E*A - B*D), (C*D - A*F)/(B*D - A*E))
    return pos


i = 0
while(True):

    try:
        line = ser.readline().decode("UTF-8")
        rad.write(line) 
        radialPos=[float(i) for i in line.split(',')] 
        datx, daty = getPos(anchor1Pos, anchor2Pos, anchor3Pos,radialPos[0]+anchorOffsets[0], radialPos[1]+anchorOffsets[1], radialPos[2]+anchorOffsets[2])
        print(str(datx) + ',' + str(daty))
        log.write(str(datx) + ',' + str(daty) + '\n')
    except Exception as e:
        print(e)
