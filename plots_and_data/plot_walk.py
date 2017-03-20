

from numpy import *
from matplotlib.pyplot import *

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    ret[n - 1:] /= n
    ret[:n-1] = ret[:n-1] / np.arange(1, n)
    return ret

path = ''
data1 = np.loadtxt(path+'walk_good.csv', dtype='int', delimiter=' ')

t = data1[:,0]
mpage = data1[:,1]
mstep = data1[:,2]  
x = data1[:,3]
y = data1[:,4]

# smoot_factor = 1
# x1 = moving_average(x1, smoot_factor);
# x2 = moving_average(x2, smoot_factor);
# y2 = moving_average(y2, smoot_factor);
# x3 = moving_average(x3, smoot_factor);
# y3 = moving_average(y3, smoot_factor);
# x4 = moving_average(x4, smoot_factor);
# y4 = moving_average(y4, smoot_factor);


figure(1)
subplot(1,2,1)
plot(t - t[0], 1279-x, label='x')
legend(loc='upper left')

subplot(1,2,2)
plot(t - t[0], 1279-y, label='y')
legend(loc='upper left')

print "motion pages:", unique(mpage) # fwd walking cycle is motion pages 36-39

fwd_cycles = []
last_page = 0
cycle_arr = None
in_cycle = False

for i in range(len(mpage)):
    if last_page != 36 and mpage[i] == 36:
        # new cycle
        in_cycle = True
        cycle_arr = []
        fwd_cycles.append(cycle_arr)
    
    if in_cycle:
        if mpage[i] in (36, 37, 38, 39):
            # continue cycle
            cycle_arr.append(i)
        else:
            in_cycle = False
    else:
        pass
    last_page = mpage[i]


figure(2)

for i, cycle in enumerate(fwd_cycles):
     # skip last one as it is incomplete
    if i == 4:
        break
    subplot(2, 2, 1+i)
    plot(t[cycle] - t[cycle][0], 1279-x[cycle], label='x cycle %d'%(i,))
    plot(t[cycle] - t[cycle][0], 1279-y[cycle], label='y cycle %d'%(i,))
    legend(loc='upper left')




## make average of 4 first fwd walk cucles

x_avg = copy(x[fwd_cycles[2]])
y_avg = copy(y[fwd_cycles[2]])
t_avg = t[fwd_cycles[2]]
use_cycles = [0,1,2,3]
for c in use_cycles:
    print "n samples in cycle", c, "is", len(x[fwd_cycles[c]])
    if c is 2:
        continue
    x_avg = x_avg + x[fwd_cycles[c]][:len(t_avg)]
    y_avg = y_avg + x[fwd_cycles[c]][:len(t_avg)]

x_avg /= len(use_cycles)
y_avg /= len(use_cycles)


figure(3)
subplot(121)
plot(t_avg  - t_avg[0], 1279-x_avg, label='x avg')
legend(loc='upper left')
subplot(122)
plot(t_avg  - t_avg[0], 1279-y_avg, label='y avg')
legend(loc='upper left')
    

show()

print '''mtn_rot_vel_timings wlk_r1[] = 
{'''
for i in range(len(t_avg)):
    print '    {%d, %d, %d},' % (t_avg[i]-t_avg[0], 1279-x_avg[i], 1279-y_avg[i])
print '};\n'
