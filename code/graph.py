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
xq=np.array([]);
yq=np.array([]);
zq=np.array([]);
xr=np.array([]);
yr=np.array([]);
zr=np.array([]);
t= [];
alphap=np.array([]);
gamma=np.array([]);
vp=np.array([]);
vpused=np.array([]);
psi=np.array([]);
theta=np.array([]);
vt=np.array([]);
alphat=np.array([]);
Rxy=np.array([]);
Rz=np.array([])
Rdotxy=np.array([]);
Rdotz=np.array([]);
S1=np.array([]);
S2=np.array([]);
S3=np.array([]);
vpactual=np.array([]);
alphapactual=np.array([])
gammaactual=np.array([]);
k1=np.array([]);
k2=np.array([]);
k3=np.array([]);
ka=np.array([]);
kc=np.array([]);
kb=np.array([]);
slidetime = 0;
varforS3=np.array([]);
simtime=np.array([]);


matplotlib.rcParams.update({'font.size': 20})

# f = open("nonman_pos.txt", "r+");
# f = open("constman_pos.txt", "r+");
# f = open("sinuman_pos.txt", "r+");
# f = open("sinusoidal1.txt", "r+");
f = open("two_bot_pos.txt", "r+");
# f = open("stationary_pos.txt", "r+");
# f = open("pos_circular.txt", "r+");
# f = open("pos_sinusoidal.txt", "r+");
# f2 = open("stationary_pos.txt", "w");
# f2 = open("nonman_pos.txt", "w");
# f2 = open("constman_pos.txt", "w");
# f2 = open("sinuman_pos.txt", "w");
f2 = open("blank.txt", "w");
for line in f:
	f2.write(line)
	arr = np.array(line.split(" "));
	# print(arr)
	arr  = arr.astype(np.float);
	if arr[14]<=0.3 and slidetime==0:# and arr[15]>=-0.8 and arr[15]<0 and slidetime==0:
		slidetime = arr[6];
	if arr[14]<=0.7 and (arr[15])>=-0.18:
		break;
	xq=np.append(xq,arr[0]);
	yq= np.append(yq,arr[1]);
	zq= np.append(zq,arr[2]);
	xr=np.append(xr,arr[3]);
	yr=np.append(yr,arr[4]);
	zr=np.append(zr,arr[5]);
	t=np.append(t,arr[6]);
	alphap=np.append(alphap,arr[7]);
	gamma=np.append(gamma,arr[8]);
	vpused=np.append(vpused,arr[9]);
	psi=np.append(psi,arr[10]);
	theta=np.append(theta,arr[11]);
	vt=np.append(vt,arr[12]);
	alphat=np.append(alphat,arr[13]);
	Rxy=np.append(Rxy,arr[14]);
	Rz=np.append(Rz,arr[15]);
	Rdotxy=np.append(Rdotxy,arr[16]);
	Rdotz=np.append(Rdotz,arr[17]);
	S1=np.append(S1,arr[18]);
	S2=np.append(S2,arr[19]);
	S3=np.append(S3,arr[20]);
	vpactual=np.append(vpactual,arr[21]);
	alphapactual=np.append(alphapactual,arr[22]);
	gammaactual=np.append(gammaactual,arr[23]);
	vp=np.append(vp,arr[24]);
	k1=np.append(k1,arr[25]);
	k2=np.append(k2,arr[26]);
	k3=np.append(k3,arr[27]);
	ka=np.append(ka,arr[28]);
	kc=np.append(kc,arr[29]);
	kb=np.append(kb,arr[30]);
	varforS3=np.append(varforS3,arr[31]);
	simtime=np.append(simtime,arr[32]);
f2.close();
	# print(vpused)
print(min(Rxy));

Rxydotactual = vt*np.cos(alphat-psi) - vp*np.cos(gammaactual)*np.cos(alphapactual-psi)
Rzdotactual = - vp*np.sin(gammaactual)
psidotactual = np.divide(1,Rxy)*(vt*np.sin(alphat-psi) - vp*np.cos(gammaactual)*np.sin(alphapactual-psi))
S1_actual = Rxydotactual + ka*Rxy
S2_actual = (Rzdotactual - Rxydotactual) + kb*(Rz - Rxy)
S3_actual = S3


# np.delete(xq,0);
# np.delete(yq,0);
# np.delete(zq,0);
# np.delete(xr,0);
# np.delete(yr,0);
# np.delete(zr,0);
# np.delete(t,0);
# np.delete(alphap,0);
# np.delete(gamma,0);
# np.delete(vp,0);
# np.delete(vpused,0);
# np.delete(psi,0);
# np.delete(theta,0);
# np.delete(vt,0);
# np.delete(alphat,0);
# np.delete(Rxy,0);
# np.delete(Rz,0);
# np.delete(Rdotxy,0);
# np.delete(Rdotz,0)
# np.delete(S1,0);
# np.delete(S2,0);
# np.delete(S3,0);
# np.delete(vpactual,0);
# np.delete(alphapactual,0);
# np.delete(gammaactual,0);
# np.delete(k1,0);
# np.delete(k2,0);
# np.delete(k3,0);
# np.delete(ka,0);
# np.delete(kc,0);
# np.delete(kb,0);
# np.delete(varforS3,0);
# np.delete(simtime,0);

slidetime=0;


damntime=0;
simtime = simtime - simtime[0];
print(t)
print(slidetime)
slideindex = len(t)-1;
for i in range(len(t)):
	if t[i] == slidetime:
		slidetime = simtime[i];
		slideindex = i;
		break;

tiiming = t;
t=simtime;
switcharray=np.array([]);
twoph=np.array([]);

for i in range(len(t)):
	if t[i] >= 35.5781 and t[i]<= 35.6332:
		damntime = i;
	if i>0 and Rxy[i]<7.5 and Rxy[i-1]>=7.5:
		twoph=np.append(twoph,t[i]);
	elif i>0 and ka[i] != ka[i-1]:
		switcharray=np.append(switcharray,t[i]);

print("damntime is",damntime);
print("twoph is",twoph);
print("switcharray is",switcharray);
print("shape ka", ka.shape)


if twoph.size ==0:
	twoph = np.append(twoph,0)
if switcharray.size==0:
	switcharray=np.append(switcharray,0)
	switcharray=np.append(switcharray,0)


# print("working till here \n");
ax.plot([xr[-1] + sqrt(2)*cos(3*pi/4-alphat[-1]),xr[-1]-sqrt(2)*cos(3*pi/4-alphat[-1]),xr[-1]-sqrt(2)*cos(3*pi/4-alphat[-1]),xr[-1]+sqrt(2)*cos(3*pi/4-alphat[-1]),xr[-1] + sqrt(2)*cos(3*pi/4-alphat[-1])],[yr[-1] + sqrt(2)*sin(3*pi/4-alphat[-1]),yr[-1] + sqrt(2)*sin(3*pi/4-alphat[-1]),yr[-1] - sqrt(2)*sin(3*pi/4-alphat[-1]),yr[-1] - sqrt(2)*sin(3*pi/4-alphat[-1]),yr[-1] + sqrt(2)*sin(3*pi/4-alphat[-1])],[zr[-1],zr[-1],zr[-1],zr[-1],zr[-1]],'--k');
ax.plot(xr,yr,zr,'-r.',label="UGV");
ax.plot(xq,yq,zq,'-b.',label="UAV");
plt.legend(loc="upper left", bbox_to_anchor=(1,1))
plt.title("Trajectory plot")
# plt.xticks(np.arange(20))
# plt.yticks(np.arange(20))
ax.set_xlabel("x axis", fontsize=18);
ax.set_ylabel("y axis", fontsize=18);
ax.set_zlabel("z axis", fontsize=18);
ax.set_aspect('equal');
# ax.auto_scale_xyz([-5, 25], [-18, 12	], [0, 25])
# ax.auto_scale_xyz([-10, 25], [-2, 33], [-10, 23])
# ax.auto_scale_xyz([0, 160], [-80, 80], [-10, 30])
ax.tick_params(labelsize=6)
plt.tight_layout()
plt.show(block=False);


# fig=plt.figure();
# plt.plot(t[0:slideindex],alphap[0:slideindex],'-b',label=r"$\alpha_{p}$");
# plt.plot(t[0:slideindex],vpused[0:slideindex],'-g',label=r"$v_{p}$");
# plt.plot(t[0:slideindex],gamma[0:slideindex],'-r',label=r"$\gamma$");
# plt.plot([slidetime, slidetime],[-np.max(vpused), np.max(vpused)],'-.k')
# plt.legend(loc="upper left", bbox_to_anchor=(1,1)
# ) 
# plt.ylabel("Used Speed and Heading angles");
# plt.xlabel("time (sec)")
# plt.tight_layout()
# plt.show(block=False);

fig=plt.figure();
plt.plot(t[0:slideindex],alphat[0:slideindex],'-b',label=r"$\alpha_{t}$");
plt.plot(t[0:slideindex],varforS3[0:slideindex],'-g',label=r"$\psi - \psi_{des}$")
plt.plot(t[0:slideindex],psi[0:slideindex],'--b',label=r"$\psi$");
plt.plot(t[0:slideindex],theta[0:slideindex],'--g',label=r"$\theta$")
plt.plot([slidetime, slidetime],[-pi, pi],'-.k')
plt.plot([switcharray[0], switcharray[0]],[-pi, pi],':ok',label=r"$t_{Stage \ 1 \ to \ 2}$")
plt.plot([switcharray[1], switcharray[1]],[-pi, pi],':ob', label=r"$t_{Stage \ 2 \ to \ 3}$")
plt.plot([twoph[0],twoph[0]],[-pi, pi],':sr', label=r"$t_{Phase \ 1 \ to \ 2}$")


plt.legend(loc="upper left", bbox_to_anchor=(1,1)
) 
plt.ylabel("Reference Angles");
plt.xlabel("time (sec)")
plt.tight_layout()
plt.show(block=False);

# fig=plt.figure();
# plt.plot(t,simtime,'-b',label=r"$simtime$");
# plt.xlabel("time (sec)")
# plt.ylabel("time (sec)")
# plt.tight_layout()
# plt.show(block=False);


# fig=plt.figure();
# plt.plot(t,alphat,'-b',label=r"$\alpha_{p}$");
# plt.plot(t,vp,'-g',label=r"$v_{p}$");
# plt.plot(t,gamma,'-r',label=r"$\gamma$");
# plt.plot([slidetime, slidetime],[-np.max(vp), np.max(vp)],'-.k')
# plt.legend(loc="upper left", bbox_to_anchor=(1,1)
# ) 
# plt.ylabel("Speed and Heading angles");
# plt.xlabel("time (sec)")
# plt.tight_layout()
# plt.show(block=False);



fig=plt.figure();
# plt.plot(t,alphat,'--b',label=r"$\alpha_{t}$");
# plt.plot(t,vt,'--g',label=r"$v_{t}$");
plt.plot(t[0:slideindex],alphapactual[0:slideindex],'-b',label=r"$\alpha_{p}$");
plt.plot(t[0:slideindex],vpactual[0:slideindex],'-g',label=r"$v_{p}$");
plt.plot(t[0:slideindex],gammaactual[0:slideindex],'-r',label=r"$\gamma$");
maxval = max([pi,max(vpactual)]);
plt.plot([switcharray[0], switcharray[0]],[-pi, maxval],':ok',label=r"$t_{Stage \ 1 \ to \ 2}$")
plt.plot([switcharray[1], switcharray[1]],[-pi, maxval],':ob', label=r"$t_{Stage \ 2 \ to \ 3}$")
plt.plot([twoph[0],twoph[0]],[-pi, maxval],':sr', label=r"$t_{Phase \ 1 \ to \ 2}$")
plt.plot([slidetime, slidetime],[-np.max(vpactual), np.max(vpactual)],'-.k')
plt.legend(loc="upper left", bbox_to_anchor=(1,1)
) 
plt.ylabel("Actual Speed and Heading angles");
plt.xlabel("time (sec)")
plt.tight_layout()
plt.show(block=False);

fig=plt.figure();
plt.plot(t[0:slideindex],k1[0:slideindex],'-b',label=r"$k_{1}$");
plt.plot(t[0:slideindex],k2[0:slideindex],'-g',label=r"$k_{2}$");
plt.plot(t[0:slideindex],k3[0:slideindex],'-r',label=r"$k_{3}$");
plt.plot(t[0:slideindex],ka[0:slideindex],'--b',label=r"$k_{a}$");
plt.plot(t[0:slideindex],kc[0:slideindex],'--g',label=r"$k_{b}$");
plt.plot(t[0:slideindex],kb[0:slideindex],'--g',label=r"$k_{c}$");
maxval=np.max([np.max(k1[0:slideindex]),np.max(k2[0:slideindex]),np.max(k3[0:slideindex]),np.max(ka[0:slideindex]),np.max(kb[0:slideindex]),np.max(kc[0:slideindex])]);
plt.plot([switcharray[0], switcharray[0]],[0, maxval],':ok',label=r"$t_{Stage \ 1 \ to \ 2}$")
plt.plot([switcharray[1], switcharray[1]],[0, maxval],':ob', label=r"$t_{Stage \ 2 \ to \ 3}$")
plt.plot([twoph[0],twoph[0]],[0, maxval],':sr', label=r"$t_{Phase \ 1 \ to \ 2}$")
plt.plot([slidetime, slidetime],[-maxval,maxval],'-.k')
plt.legend(loc="upper left", bbox_to_anchor=(1,1)
) 
plt.ylabel("tuning params");
plt.xlabel("time (sec)")
plt.tight_layout()
plt.show(block=False);


fig=plt.figure();
plt.plot(t[0:slideindex],Rxy[0:slideindex],'-b',label=r"$R_{xy}$");
plt.plot(t[0:slideindex],Rz[0:slideindex],'-r',label=r"$R_{z}$");
plt.plot(t[0:slideindex],Rdotxy[0:slideindex],'--b',label=r"$\dot{R}_{xy}$");
plt.plot(t[0:slideindex],Rdotz[0:slideindex],'--r',label=r"$\dot{R}_{z}$");
maxval = max(Rxy);
minval = min(Rz);
plt.plot([switcharray[0], switcharray[0]],[minval, maxval],':ok',label=r"$t_{Stage \ 1 \ to \ 2}$")
plt.plot([switcharray[1], switcharray[1]],[minval, maxval],':ob', label=r"$t_{Stage \ 2 \ to \ 3}=$" + '\n' + str(np.round(switcharray[1],1)) + "s")
plt.plot([twoph[0],twoph[0]],[minval, maxval],':sr', label=r"$t_{Phase \ 1 \ to \ 2}=$" + '\n' + str(np.round(twoph[0],1)) + "s")
plt.plot([slidetime, slidetime],[-np.max(Rxy), np.max(Rxy)],'-.k')
# plt.plot(t,theta,'--g',label=r"$\theta$");
plt.legend(loc="upper left", bbox_to_anchor=(1,1)
) 
plt.ylabel("Displacements and their rates");
plt.xlabel("time (sec)")
plt.tight_layout()
plt.show(block=False);

fig=plt.figure();
plt.plot(t[0:slideindex],S1[0:slideindex],'-b',label=r"$S_{1}$");
plt.plot(t[0:slideindex],S2[0:slideindex],'-g',label=r"$S_{2}$");
plt.plot(t[0:slideindex],S3[0:slideindex],'-r',label=r"$S_{3}$");
maxval=np.max([np.max(S1),-np.min(S1),np.max(S2),-np.min(S2),np.max(S3),-np.min(S3)]);
plt.plot([switcharray[0], switcharray[0]],[-maxval, maxval],':ok',label=r"$t_{Stage \ 1 \ to \ 2}$")
plt.plot([switcharray[1], switcharray[1]],[-maxval, maxval],':ob', label=r"$t_{Stage \ 2 \ to \ 3}$")
plt.plot([twoph[0],twoph[0]],[-maxval, maxval],':sr', label=r"$t_{Phase \ 1 \ to \ 2}$")
plt.plot([slidetime, slidetime],[-maxval, maxval],'-.k')
# plt.plot(t,Rdotz,'--b',label=r"$\dot{R}_{z}$");
# plt.plot(t,theta,'--g',label=r"$\theta$");
plt.legend(loc="upper left", bbox_to_anchor=(1,1)
) 
plt.ylabel("Sliding variables");
plt.xlabel("time (sec)")
plt.tight_layout()
plt.show();

# f.close();
# print(np.shape(xq));
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