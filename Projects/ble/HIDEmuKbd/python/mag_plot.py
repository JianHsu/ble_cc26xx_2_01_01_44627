from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

ax = []
ay = []
az = []
ax_c = []
ay_c = []
az_c = []

mx = []
my = []
mz = []
mx_c = []
my_c = []
mz_c = []

with open("mag.txt", "r") as ins:
    for line in ins:
        arr = line.split(',')
        if (len(arr) == 10) :
        	ax.append(float(arr[0]) * 1000)
        	ay.append(float(arr[1]) * 1000)
        	az.append(float(arr[2]) * 1000)
        	mx.append(float(arr[6]))
        	my.append(float(arr[7]))
        	mz.append(float(arr[8]))

amax = [max(ax), max(ay), max(az)]
amin = [min(ax), min(ay), min(az)]
aavg = []
aavg.append((amax[0] + amin[0]) / 2)
aavg.append((amax[1] + amin[1]) / 2)
aavg.append((amax[2] + amin[2]) / 2)
#print amax
#print amin
#print aavg
for i in range(len(ax)):
	ax_c.append(ax[i] - aavg[0])
	ay_c.append(ay[i] - aavg[1])
	az_c.append(az[i] - aavg[2])


mmax = [max(mx), max(my), max(mz)]
mmin = [min(mx), min(my), min(mz)]
mavg = []
mavg.append((mmax[0] + mmin[0]) / 2)
mavg.append((mmax[1] + mmin[1]) / 2)
mavg.append((mmax[2] + mmin[2]) / 2)
print mmax
print mmin
print mavg
for i in range(len(mx)):
	mx_c.append(mx[i] - mavg[0])
	my_c.append(my[i] - mavg[1])
	mz_c.append(mz[i] - mavg[2])

fig = plt.figure()
#ax = fig.add_subplot(241, projection='3d')
mxy = fig.add_subplot(331)
mxz = fig.add_subplot(332)
myz = fig.add_subplot(333)
mxy_c = fig.add_subplot(334)
mxz_c = fig.add_subplot(335)
myz_c = fig.add_subplot(336)
axy = fig.add_subplot(337)
axz = fig.add_subplot(338)
ayz = fig.add_subplot(339)
#axy_c = fig.add_subplot(4,3,10)
#axz_c = fig.add_subplot(4,3,11)
#ayz_c = fig.add_subplot(4,3,12)
#ax.scatter(mx, my, mz)

#ax.set_xlabel('X Label')
#ax.set_ylabel('Y Label')
#ax.set_zlabel('Z Label')

axy.plot(ax, ay)
axy.set_xlabel('aX')
axy.set_ylabel('aY')
axy.axis([-3000, 3000, -3000, 3000])
axz.plot(ax, az)
axz.set_xlabel('aX')
axz.set_ylabel('aZ')
axz.axis([-3000, 3000, -3000, 3000])
ayz.plot(ay, az)
ayz.set_xlabel('aY')
ayz.set_ylabel('aZ')
ayz.axis([-3000, 3000, -3000, 3000])
'''
axy_c.plot(ax_c, ay_c)
axy_c.set_xlabel('aX')
axy_c.set_ylabel('aY')
axy_c.axis([-3000, 3000, -3000, 3000])
axz_c.plot(ax_c, az_c)
axz_c.set_xlabel('aX')
axz_c.set_ylabel('aZ')
axz_c.axis([-3000, 3000, -3000, 3000])
ayz_c.plot(ay_c, az_c)
ayz_c.set_xlabel('aY')
ayz_c.set_ylabel('aZ')
ayz_c.axis([-3000, 3000, -3000, 3000])
'''
mxy.plot(mx, my)
mxy.set_xlabel('mX')
mxy.set_ylabel('mY')
mxy.axis([-1000, 1000, -1000, 1000])
mxz.plot(mx, mz)
mxz.set_xlabel('mX')
mxz.set_ylabel('mZ')
mxz.axis([-1000, 1000, -1000, 1000])
myz.plot(my, mz)
myz.set_xlabel('mY')
myz.set_ylabel('mZ')
myz.axis([-1000, 1000, -1000, 1000])


mxy_c.plot(mx_c, my_c)
mxy_c.set_xlabel('mX')
mxy_c.set_ylabel('mY')
mxy_c.axis([-500, 500, -500, 500])
mxz_c.plot(mx_c, mz_c)
mxz_c.set_xlabel('mX')
mxz_c.set_ylabel('mZ')
mxz_c.axis([-500, 500, -500, 500])
myz_c.plot(my_c, mz_c)
myz_c.set_xlabel('mY')
myz_c.set_ylabel('mZ')
myz_c.axis([-500, 500, -500, 500])

plt.show()