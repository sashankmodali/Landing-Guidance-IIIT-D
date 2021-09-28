#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import rc
from math import *
import numpy as np
import matplotlib.animation as animation


fig = plt.figure()
plt.tight_layout()
ax1 = fig.add_subplot(2,2,1)
ax2 = fig.add_subplot(2,2,2)
ax3 = fig.add_subplot(2,2,3)
ax4 = fig.add_subplot(2,2,4)
# rc('font',size=18)
# rc('axes',labelsize=18)

# xr,yr,np.sqrt(v_xr**2 + v_yr**2),yawr, v_yaw_r,X[0,0],X[1,0],X[4,0],X[2,0],X[3,0],simtime
# ax = fig.add_subplot(111, projection='3d')

def animate(i):
	xr=np.array([]);
	yr=np.array([]);
	vr = np.array([]);
	yawr=np.array([]);
	v_yaw_r=np.array([]);
	xe=np.array([]);
	ye= np.array([]);
	ve=np.array([]);
	yawe=np.array([]);
	v_yaw_e=np.array([]);
	simtime=np.array([]);


	matplotlib.rcParams.update({'font.size': 10})

	f = open("rover_kalman.txt","r");
	f2 = open("blankestimator.txt", "w");
	for line in f:
		f2.write(line)
		arr = np.array(line.split(" "));
		arr  = arr.astype(np.float);
		xr=np.append(xr,arr[0]);
		yr=np.append(yr,arr[1]);
		vr = np.append(vr,arr[2]);
		yawr=np.append(yawr,arr[3]);
		v_yaw_r=np.append(v_yaw_r,arr[4]);
		xe=np.append(xe,arr[5]);
		ye= np.append(ye,arr[6]);
		ve=np.append(ve,arr[7]);
		yawe=np.append(yawe,arr[8]);
		v_yaw_e=np.append(v_yaw_e,arr[9]);
		simtime=np.append(simtime,arr[10]);
	f2.close();
	f.close()
		# print(vpused)

	simtime = simtime - simtime[0];

	# tiiming = t;
	# t=simtime;
	history = len(simtime)
	# print("working till here \n");
	ax2.clear();
	ax2.plot(simtime[-min([history,len(simtime)]):-1],yr[-min([history,len(simtime)]):-1],'--r',label="True");
	ax2.plot(simtime[-min([history,len(simtime)]):-1],ye[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax2.legend();
	# plt.legend(loc="upper left", bbox_to_anchor=(1,1))
	ax2.set_title("y plot")
	ax2.set_xlabel("time", fontsize=18);
	ax2.set_ylabel("y axis", fontsize=18);
	# ax2.set_ylim([min(yr[-min([history,len(simtime)]):-1])-5,max(yr[-min([history,len(simtime)]):-1])+5])
	# ax = plt.gca()
	# ax.set_aspect('equal');
	# ax.tick_params(labelsize=6)
	# ax1.set_tight_layout()
	# plt.show(block=False);

	ax1.clear();
	ax1.plot(simtime[-min([history,len(simtime)]):-1],xr[-min([history,len(simtime)]):-1],'--r',label="True");
	ax1.plot(simtime[-min([history,len(simtime)]):-1],xe[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax1.legend();#(loc="upper left", bbox_to_anchor=(1,1))
	ax1.set_title("x plot")
	ax1.set_xlabel("time", fontsize=18);
	ax1.set_ylabel("x axis", fontsize=18);


	ax3.clear();
	ax3.plot(simtime[-min([history,len(simtime)]):-1],vr[-min([history,len(simtime)]):-1],'--r',label="True");
	ax3.plot(simtime[-min([history,len(simtime)]):-1],ve[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax3.legend();#(loc="upper left", bbox_to_anchor=(1,1))
	ax3.set_title("x plot")
	ax3.set_xlabel("time", fontsize=18);
	ax3.set_ylabel("x axis", fontsize=18);

	ax4.clear();
	ax4.plot(simtime[-min([history,len(simtime)]):-1],v_yaw_r[-min([history,len(simtime)]):-1],'--r',label="True");
	ax4.plot(simtime[-min([history,len(simtime)]):-1],v_yaw_e[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax4.legend();#(loc="upper left", bbox_to_anchor=(1,1))
	ax4.set_title("x plot")
	ax4.set_xlabel("time", fontsize=18);
	ax4.set_ylabel("x axis", fontsize=18);
	# ax1.set_ylim([min(xr[-min([history,len(simtime)]):-1])-5,max(xr[-min([history,len(simtime)]):-1])+5])
	# ax = plt.gca()
	# ax.set_aspect('equal');
	# ax.tick_params(labelsize=6)
	# ax2.tight_layout()

ani = animation.FuncAnimation(fig, animate, interval=500)
plt.show()