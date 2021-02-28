#%%
import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("/Users/byeongilham/Documents/C/cv1.txt",dtype=None)
t = data[:,0]
ori_pos = data[:,1]
noise_pos = data[:,2]
filtered_pos = data[:,3]

ori_vel = data[:,4]
noise_vel = data[:,5]
filtered_vel = data[:,6]


plt.figure(num=2,dpi=550,facecolor='white')

plt.subplot(2,1,1)
plt.plot(t,ori_pos)
#plt.plot(t,noise_pos)
plt.plot(t,filtered_pos)
plt.xlabel('time[s]')
plt.ylabel('position[m]')
plt.legend(['ori_pos','filtered_pos'])
plt.show

plt.subplot(2,1,2)
plt.plot(t,ori_vel)
#plt.plot(t,noise_vel)
plt.plot(t,filtered_vel)
plt.xlabel('time[s]')
plt.ylabel('velocity[m/s]')
plt.legend(['ori_vel','filtered_vel'])

plt.show
# %%
plt.subplot(2,1,1)
plt.plot(t,noise_pos)
plt.plot(t,filtered_pos)
plt.xlabel('time[s]')
plt.ylabel('position[m]')
plt.legend(['noise_pos','filtered_pos'])
plt.show

plt.subplot(2,1,2)
plt.plot(t,noise_vel)
plt.plot(t,filtered_vel)
plt.xlabel('time[s]')
plt.ylabel('velocity[m/s]')
plt.legend(['noie_vel','filtered_vel'])
# %%
