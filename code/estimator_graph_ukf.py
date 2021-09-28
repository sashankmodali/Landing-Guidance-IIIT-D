#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import rc
from math import *
import numpy as np
import matplotlib.animation as animation


fig,((ax1,ax2),(ax3,ax4),(ax5,ax6), (ax7, ax8)) = plt.subplots(4,2)
fig.suptitle("UKF estimates")
# ax7 = fig.add_subplot(421,projection='3d')
# ax2 = fig.add_subplot(3,2,2)
# ax3 = fig.add_subplot(3,2,3)
# ax4 = fig.add_subplot(3,2,4)
# ax5 = fig.add_subplot(3,2,5)
# ax6 = fig.add_subplot(3,2,6)
fig.tight_layout()
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
	P_k = np.zeros((1,7,7));
	x_m = np.array([]);
	y_m = np.array([]);
	alpha = np.array([]);
	acc = np.array([]);

	matplotlib.rcParams.update({'font.size': 10})

	f = open("rover_ukf.txt","r");
	f2 = open("blankestimator2.txt", "w");
	for line in f:
		f2.write(line)
		arr1 = np.array(line.split(" "));
		arr  = arr1.astype(np.float);
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
		# print(arr.shape)
		P_k = np.append(P_k,arr[11:11+49].reshape(1,7,7),axis=0);
		x_m = np.append(x_m, arr[11+49])
		y_m = np.append(y_m, arr[11+50])
		alpha = np.append(alpha, arr[11+51])
		acc = np.append(acc, arr[11+52])
		# print(P_k.shape)
	f2.close();
	f.close()
		# print(vpused)

	# simtime = simtime - simtime[0];

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
	ax2.set_xlabel("time");
	ax2.set_ylabel("y");
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
	ax1.set_xlabel("time");
	ax1.set_ylabel("x");


	ax3.clear();
	ax3.plot(simtime[-min([history,len(simtime)]):-1],vr[-min([history,len(simtime)]):-1],'--r',label="True");
	ax3.plot(simtime[-min([history,len(simtime)]):-1],ve[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax3.legend();#(loc="upper left", bbox_to_anchor=(1,1))
	ax3.set_title("v plot")
	ax3.set_xlabel("time");
	ax3.set_ylabel("v");

	ax4.clear();
	ax4.plot(simtime[-min([history,len(simtime)]):-1],np.sum(P_k[:,0:3,0:3],axis=(1,2))[-min([history,len(simtime)]):-1],'--r',label="variance");
	# ax4.plot(simtime[-min([history,len(simtime)]):-1],v_yaw_e[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax4.legend();#(loc="upper left", bbox_to_anchor=(1,1))
	ax4.set_title("variance plot")
	ax4.set_xlabel("time");
	ax4.set_ylabel("variance");
	# ax1.set_ylim([min(xr[-min([history,len(simtime)]):-1])-5,max(xr[-min([history,len(simtime)]):-1])+5])
	# ax = plt.gca()
	# ax.set_aspect('equal');
	# ax.tick_params(labelsize=6)
	# ax2.tight_layout()

	ax5.clear();
	ax5.plot(simtime[-min([history,len(simtime)]):-1],yawr[-min([history,len(simtime)]):-1],'--r',label="True");
	ax5.plot(simtime[-min([history,len(simtime)]):-1],yawe[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax5.legend();#(loc="upper left", bbox_to_anchor=(1,1))
	ax5.set_title("yaw")
	ax5.set_xlabel("time");
	ax5.set_ylabel("x_m");

	history = int(len(simtime))

	ax6.clear();
	# ax6.plot(simtime[-min([history,len(simtime)]):-1],yr[-min([history,len(simtime)]):-1],'--r',label="True");
	ax6.plot(simtime[-min([history,len(simtime)]):-1],acc[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	# ax6.plot(simtime[-min([history,len(simtime)]):-1],y_m[-min([history,len(simtime)]):-1],'--k',label="Meas dur");
	ax6.legend();#(loc="upper left", bbox_to_anchor=(1,1))
	ax6.set_title("accel")
	ax6.set_xlabel("time");
	ax6.set_ylabel("a");

	ax7.clear();
	ax7.plot(simtime[-min([history,len(simtime)]):-1],v_yaw_r[-min([history,len(simtime)]):-1],'--r',label="True");
	ax7.plot(simtime[-min([history,len(simtime)]):-1],v_yaw_e[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax7.set_title("yaw rate")
	ax7.set_xlabel("time");
	ax7.set_ylabel("om");

	ax8.clear();
	ax8.plot(simtime[-min([history,len(simtime)]):-1],alpha[-min([history,len(simtime)]):-1],'-b',label="Estimated");
	ax8.set_title("ang accel")
	ax8.set_xlabel("time");
	ax8.set_ylabel("alpha");

ani = animation.FuncAnimation(fig, animate, interval=100000)
plt.show()