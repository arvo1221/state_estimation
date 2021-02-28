#%%
import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("/Users/byeongilham/Documents/C/cv2.txt",dtype=None)
t = data[:,12]
ori_pos_x = data[:,0]
noise_pos_x = data[:,2]
filtered_pos_x = data[:,4]
ori_pos_y = data[:,1]
noise_pos_y = data[:,3]
filtered_pos_y = data[:,5]


ori_vel_x = data[:,6]
noise_vel_x = data[:,8]
filtered_vel_x = data[:,10]
ori_vel_y = data[:,7]
noise_vel_y = data[:,9]
filtered_vel_y = data[:,11]

plt.figure(figsize = (10,10),dpi=600,facecolor='white')

plt.subplot(3,1,1)
plt.plot(ori_pos_x,ori_pos_y)
plt.plot(filtered_pos_x,filtered_pos_y)
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.legend(['origin_pos','filtered_pos'])
plt.show

plt.subplot(3,1,2)
plt.plot(t,ori_vel_x)
plt.plot(t,filtered_vel_x)
plt.xlabel('time[s]')
plt.ylabel('x axis velocity[m/s]')
plt.legend(['origin_vel','filtered_vel'])

plt.subplot(3,1,3)
plt.plot(t,ori_vel_y)
plt.plot(t,filtered_vel_y)
plt.xlabel('time[s]')
plt.ylabel('y axis velocity[m/s]')
plt.legend(['origin_vel','filtered_vel'])

plt.show

# %%
