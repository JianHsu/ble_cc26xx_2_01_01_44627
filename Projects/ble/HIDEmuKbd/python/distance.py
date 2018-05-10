from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

ax = []
ax_c = []
aavg = 0
vx = []
vx_c = []
ay = []
az = []
t = []
t_cnt = 1
with open("distance.txt", "r") as ins:
    for line in ins:
        arr = line.split(',')
        if (len(arr) == 10):
            ax.append(float(arr[0]) * 9.81)
            aavg += float(arr[0]) * 9.81
            ay.append(float(arr[1]) * 1000)
            az.append(float(arr[2]) * 1000)            
            t.append(float(arr[9]) * t_cnt)
            t_cnt += 1

aavg = aavg / t_cnt
print aavg

for a in ax:
    ax_c.append(a - aavg);

for a in ax:
    if (len(vx) > 1):
        vx.append(a + vx[len(vx) - 2])
    else:
        vx.append(a)

for a in ax_c:
    if (len(vx_c) > 1):
        vx_c.append(a + vx_c[len(vx_c) - 2])
    else:
        vx_c.append(a)

fig = plt.figure()

fx = fig.add_subplot(311)
fy = fig.add_subplot(312)
fz = fig.add_subplot(313)

fx.plot(t, ax)
#fx.plot(t, ax_c)
fx.set_xlabel('time')
fx.set_ylabel('aX')
fx.axis([1, 10, -10, 10])

fy.plot(t, vx)
#fy.plot(t, vx_c)
fy.set_xlabel('time')
fy.set_ylabel('aY')
fy.axis([0, 10, -200, 200])

fz.plot(t, az)
fz.set_xlabel('time')
fz.set_ylabel('aZ')
fz.axis([0, 15, -1000, 1000])




plt.show()