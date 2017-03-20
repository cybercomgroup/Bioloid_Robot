

from numpy import *
from matplotlib.pyplot import *

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    ret[n - 1:] /= n
    ret[:n-1] = ret[:n-1] / np.arange(1, n)
    return ret

path = ''
data1 = np.loadtxt(path+'gyro_data1.csv', dtype='uint32', delimiter=' ')
data2 = np.loadtxt(path+'gyro_data2.csv', dtype='uint32', delimiter=' ')
data3 = np.loadtxt(path+'gyro_data3.csv', dtype='uint32', delimiter=' ')
data4 = np.loadtxt(path+'gyro_data4.csv', dtype='uint32', delimiter=' ')

t1 = data1[:,0]/1000.0
x1 = data1[:,1]

t2 = data2[:,0]/1000.0
x2 = data2[:,1]
y2 = data2[:,2]

t3 = data3[:,0]/1000.0
x3 = data3[:,1]
y3 = data3[:,2]

t4 = data4[:,0]/1000.0
x4 = data4[:,1]
y4 = data4[:,2]

smoot_factor = 1
x1 = moving_average(x1, smoot_factor);
x2 = moving_average(x2, smoot_factor);
y2 = moving_average(y2, smoot_factor);
x3 = moving_average(x3, smoot_factor);
y3 = moving_average(y3, smoot_factor);
x4 = moving_average(x4, smoot_factor);
y4 = moving_average(y4, smoot_factor);


figure(1)
plot(t1, x1, label='x1')

plot(t2, x2, label='x2')
plot(t2, y2, label='y2')

plot(t3, x3, label='x3')
plot(t3, y3, label='y3')

plot(t4, x4, label='x4')
plot(t4, y4, label='y4')

legend(loc='upper left')


figure(2)
# plot(t1[:-1], diff(x1)/diff(t1), label='x1')
# 
# plot(t2[:-1], diff(x2)/diff(t2), label='x2')
# plot(t2[:-1], diff(y2)/diff(t2), label='y2')
# 
# plot(t3[:-1], diff(x3)/diff(t3), label='x3')
# plot(t3[:-1], diff(y3)/diff(t3), label='y3')
# 
# plot(t4[:-1], diff(x4)/diff(t4), label='x4')
# plot(t4[:-1], diff(y4)/diff(t4), label='y4')


tmp = arange(0, 100, 0.1)
tmp_dt = 0.1
tmp[0] = 8
for i in range(1, len(tmp)):
    tmp[i] = tmp[i-1] - tmp[i-1]*440/10000 * tmp_dt
plot(arange(0, 100, 0.1), tmp)


tmp2 = arange(0, 100, 0.1)
tmp_dt = 0.1
tmp2[0] = 8
for i in range(1, len(tmp2)):
    tmp2[i] = tmp2[i-1] - tmp2[i-1]*429/10000 * tmp_dt
plot(arange(0, 100, 0.1), tmp2)

legend(loc='upper left')


figure(1)

tmpx = arange(0, 100, 0.1)
tmp_dt = 0.1
tmpx[0] = 1100
for i in range(1, len(tmpx)): 
    tmpx[i] = tmpx[i-1] + tmp[i] * tmp_dt
    
plot(arange(0, 100, 0.1) - 2, tmpx)
plot(arange(0, 100, 0.1) - 1, tmpx)
plot(arange(0, 100, 0.1) - 7, tmpx)

tmp_dt = 0.1
tmpy = arange(0, 100, tmp_dt)
tmpy[0] = 1100
for i in range(1, len(tmpy)): 
    tmpy[i] = tmpy[i-1] + tmp2[i] * tmp_dt

plot(arange(0, 100, 0.1) - 3, tmpy)
plot(arange(0, 100, 0.1) - 8.5, tmpy)


tmp_dt = 0.1
tmpy = arange(3, 100, tmp_dt)
tmpy[0] = 1171
for i in range(1, len(tmpy)): 
    tmpy[i] = tmpy[i-1] + tmp2[i] / (1/tmp_dt)

#plot(arange(0, 100, 0.1) - 3, tmpy)
#plot(arange(0, 100, 0.1) - 8.5, tmpy)

figure(3)

## plot an x curve and corresponding predicted x curve

plot(t1, x1, label='actual x1')


# below is work in progress!! 

pred_x1 = zeros(len(t1))
dx = 0 # sample some values to determine starting dx
sampling_time = 1 # collect samples for 1 second

it = argmax(t1 - t1[0] > sampling_time)
dx = x1[it] - x1[0]
pred_x1[it] = x1[it]

for i in range(it+1, len(t1)):
    dt = t1[i] - t1[i-1]
    dx = dx - dx*440/10000 * dt # delta of x in per second
    pred_x1[i] = pred_x1[i-1] + dx * dt
plot(t1, pred_x1, label='predicted x1')

legend(loc='upper left')

show()


