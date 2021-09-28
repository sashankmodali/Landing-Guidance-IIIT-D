#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import rc
from math import *
import numpy as np


rc('font',size=18)
rc('axes',labelsize=18)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
xr=np.array([]);
yr=np.array([]);
zr=np.array([]);
t= [];
vt=np.array([]);
alphat=np.array([]);
vtactual=np.array([]);
alphatactual=np.array([]);
alphatdot=np.array([]);
alphatbody=np.array([]);

matplotlib.rcParams.update({'font.size': 18})


# f = open("sinusoidal1.txt", "r+");
f = open("rovertraj.txt", "r+");
for line in f:
	arr = np.array(line.split(" "));
	arr  = arr.astype(np.float);
	xr=np.append(xr,arr[0]);
	yr= np.append(yr,arr[1]);
	zr= np.append(zr,arr[2]);
	t=np.append(t,arr[3]);
	vt=np.append(vt,arr[4]);
	alphat=np.append(alphat,arr[6]);
	vtactual=np.append(vtactual,arr[5]);
	alphatactual=np.append(alphatactual,arr[7]);
	alphatdot=np.append(alphatdot,arr[8]);
	alphatbody=np.append(alphatbody,arr[9]);
	# print(vpused)

# print("working till here \n");
ax.plot(xr,yr,zr,'-r.',label="UGV");
ax.plot([xr[-1] + 1*cos(alphat[-1]),xr[-1]-1*cos(alphat[-1]),xr[-1]-1*cos(alphat[-1]),xr[-1]+1*cos(alphat[-1]),xr[-1] + 1*cos(alphat[-1])],[yr[-1] + 1*sin(alphat[-1]),yr[-1] + 1*sin(alphat[-1]),yr[-1] - 1*sin(alphat[-1]),yr[-1] - 1*sin(alphat[-1]),yr[-1] + 1*sin(alphat[-1])],[zr[-1],zr[-1],zr[-1],zr[-1],zr[-1]],'--k');
plt.legend(loc="upper left", bbox_to_anchor=(1,1))
plt.title("Trajectory plot")
# plt.xticks(np.arange(20))
# plt.yticks(np.arange(20))
ax.set_xlabel("x axis", fontsize=18);
ax.set_ylabel("y axis", fontsize=18);
ax.set_zlabel("z axis", fontsize=18);
# ax.set_aspect('equal');
# ax.auto_scale_xyz([0, 30], [-20, 10], [-5, 25])
# ax.auto_scale_xyz([0, 40], [-10, 30], [-10, 30])
ax.tick_params(labelsize=6)
plt.tight_layout()
plt.show(block=False);

fig=plt.figure();
plt.plot(t,np.sqrt(np.square(xr) + np.square(yr)),'-b',label=r"$\R_{xy}$");
plt.show(block=False);

fig=plt.figure();
plt.plot(t,alphat,'-b',label=r"$\alpha_{t}$");
plt.plot(t,vt,'-g',label=r"$v_{t}$");
plt.plot(t,alphatactual,'--b',label=r"$\alpha_{tactual}$");
plt.plot(t,vtactual,'--g',label=r"$v_{tactual}$");
plt.plot(t,alphatdot,'--k',label=r"$\dot{\alpha}_{tactual}$");
plt.plot(t,alphatbody,'-k',label=r"$\alpha_{tbody}$");
plt.legend(loc="upper left", bbox_to_anchor=(1,1)
) 
plt.ylabel("Speed and Heading angles");
plt.xlabel("time (sec)")
plt.tight_layout()
plt.show();

print(2*np.pi/np.mean(alphatdot))

# f=open("/home/sashankmns/SMLG_logs/sinusoidal.txt","w");
# f.write("%s\n"%(xq.tolist()));
# f.write("%s\n"%(yq.tolist()));
# f.write("%s\n"%(zq.tolist()));
# f.write("%s\n"%(xr.tolist()));
# f.write("%s\n"%(yr.tolist()));
# f.write("%s\n"%(zr.tolist()));
# f.write("%s\n"%(t.tolist()));
# f.write("%s\n"%(alphap.tolist()));
# f.write("%s\n"%(gamma.tolist()));
# f.write("%s\n"%(vp.tolist()));
# f.write("%s\n"%(psi.tolist()));
# f.write("%s\n"%(theta.tolist()));
# f.write("%s\n"%(vt.tolist()));
# f.write("%s\n"%(alphat.tolist()));
# f.write("%s\n"%(Rxy.tolist()));
# f.write("%s\n"%(Rz.tolist()));
# f.write("%s\n"%(Rdotxy.tolist()));
# f.write("%s\n"%(Rdotz.tolist()));
# f.write("%s\n"%(S1.tolist()));
# f.write("%s\n"%(S2.tolist()));
# f.write("%s\n"%(S3.tolist()));
# f.close();