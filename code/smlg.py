#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import numpy as np
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import *
# from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import datetime
import control.matlab as mb
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import time
from timeit import default_timer as timer
import timeit

class Server:
	def __init__(self):
		self.quad = Odometry();
		self.rover = Odometry();

	def land(self):
		global velpursuer, zq, zr, reached
		if abs(zr-zq)>0.01 and reached!=True:
			msg.linear.z = -sqrt(2*9.815*abs(zr-zq));
		else:
			set_mode = rospy.ServiceProxy('/quad/set_mode', mavros_msgs.srv.SetMode)
			print "set mode: ", set_mode(208,'GUIDED')
			land = rospy.ServiceProxy('/quad/cmd/arming', mavros_msgs.srv.CommandBool)
			print "land: ", land(0)
			print('landed!!!!')
			printsflag = False;
			reached = True;
	def wrapangle(self, angle):

		while abs(angle)>pi:
			angle = angle - np.sign(angle)*2*pi;

		return angle

	def rover_callback(self, dat):
		# "Store" message received.
		self.rover= dat;

		## Compute stuff.
		# self.compute_stuff()

	def quad_callback(self, dat):
		# "Store" the message received.
		self.quad = dat;

		## Compute stuff.
		# self.compute_stuff()
	
	def giveinputs(self, y,angledesired,ka,kb,kc,k1,k2,k3,alphatdot,alphatddot,vtdot, angled):
		global deltat, thetadesired, powerflag
		degreenum=3.0;
		degree=5.0;
		inputvec = np.zeros((3,1));
		alphat = y[0]; vt = y[1]; vp = y[2]; gamma =y[3];alphap=y[4];Rxy=y[5];Rz=y[6];psi=y[7];
		theta = atan2(-Rz, Rxy);
		Rdotxy = vt*cos(alphat-psi) - vp*cos(gamma)*cos(alphap-psi);
		psidot = (vt*sin(alphat-psi) - vp*cos(gamma)*sin(alphap-psi))/Rxy;
		Rdotz = -vp*sin(gamma);
		varforS3 = psi-(alphat + angledesired);
		varforS3 = self.wrapangle(varforS3);
		S = np.array([[Rdotxy + ka*Rxy],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)], [psidot - alphatdot + kb*(varforS3)]]);
		if acc=="highmaneuvering":
			varforS3 = self.wrapangle(psi-angled);
			S = np.array([[Rdotxy + ka*Rxy],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)], [psidot + kb*(varforS3)]])
		A = np.array([[-cos(alphap-psi)*cos(gamma), vp*sin(alphap-psi)*cos(gamma), vp*cos(alphap-psi)*sin(gamma)],[-sin(gamma), 0, -vp*cos(gamma)],[-sin(alphap-psi)*cos(gamma), -vp*cos(alphap-psi)*cos(gamma), vp*sin(alphap-psi)*sin(gamma)]]);
		A = np.matmul(np.array([[1, 0, 0],[tan(thetadesired), 1, 0],[0, 0, 1]]), A);

		angleterm = -kb*Rxy*(psidot-alphatdot);
		if acc=="highmaneuvering":
			angleterm = -kb*Rxy*(psidot)*(np.sign(term3));
		b1=-k1*np.sign(S[0,0])*abs(S[0,0])**(degreenum/degree) + vp*sin(alphap-psi)*cos(gamma)*psidot - vtdot*cos(alphat-psi)+vt*sin(alphat-psi)*(alphatdot-psidot) - ka*(Rdotxy)*(np.sign(term1)*(1.0/3.0*abs(term1)**(-2.0/3.0)))**powerflag;
		b2=-k2*np.sign(S[1,0])*abs(S[1,0])**(degreenum/degree) + tan(thetadesired)*(vp*sin(alphap-psi)*cos(gamma)*psidot - vtdot*cos(alphat-psi)+vt*sin(alphat-psi)*(alphatdot-psidot)) - kc*(Rdotz + tan(thetadesired)*Rdotxy)*((1.0/3.0*np.sign(term2)*abs(term2)**(-2.0/3.0)))**powerflag;
		b3=-Rxy*k3*np.sign(S[2,0])*abs(S[2,0])**(degreenum/degree) +angleterm + alphatddot*Rxy + Rdotxy*psidot - vp*cos(alphap-psi)*cos(gamma)*psidot - vt*cos(alphat-psi)*(alphatdot-psidot) - vtdot*sin(alphat-psi);
		if acc=="highmaneuvering":
			b3=-Rxy*k3*np.sign(S[2,0])*abs(S[2,0])**(degreenum/degree) +angleterm + Rdotxy*psidot - vp*cos(alphap-psi)*cos(gamma)*psidot - vt*cos(alphat-psi)*(alphatdot-psidot) - vtdot*sin(alphat-psi);
		B[0,0] = b1;
		B[1,0] = b2;
		B[2,0] = b3;
		if vp**2 >0.01 and (pi/2 - abs(gamma)) > 0.1 and ( vp>0.8 or Rxy<2 ):
			inputvec = np.matmul(np.linalg.inv(A),B);
		else:
			if vp<0.1 or (vp<0.8 and Rxy>2):
				vpvar=5;
			else:
				vpvar=0;
			if abs(self.wrapangle(pi/2-abs(gamma)))<0.01:
				gammavar=-np.sign(gamma)*pi/4;
			else:
				gammavar=0;
			inputvec = np.array([[vpvar],[0],[gammavar]]);
			
		vpdot = inputvec[0,0];
		alphapdot= inputvec[1,0];
		gammadot = inputvec[2,0];

		if abs(vpdot) > 10:
			vpdot = np.sign(vpdot)*10;
		if abs(alphapdot) > pi/2:
			alphapdot = np.sign(alphapdot)*pi/2;
		if abs(gammadot) > pi/2:
			gammadot = np.sign(gammadot)*pi/2;
		result = np.array([alphatdot,vtdot,vpdot,gammadot,alphapdot,Rdotxy,Rdotz,psidot]);
		return result;

	def compute_stuff(self,timer):
		if self.quad is not None and self.rover is not None:
			f = open("logdata.txt","a+");
			global alphato, xq, yq, zq, xr, yr, zr, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, tnow, tnow_p, tnow_p_p, error_head_prev, goal, goal_body, errpos, flag, flag3, changeflag, Rxyo, Rzo, Ro, printflag, qposarray, rposarray, fig, printsflag, landflag, velpursuer, alphap,vt, yawr, hoverdist, reached, averagevel, counter, flag2, vp, gamma, alphat, Rdotxyo, vpmax, switchcounter,tnow, angledesired, k1, k2, k3, ka, kb, S,So, A,B, vpdotprev, alphapdotprev, gammadotprev, alphatprev,alphatdprev, deltat, deltat_p , tstartalphatdot, tstartvt, vt,vtprev, Rxy, Rz, vpo, vto, acc, alphatdoto, psidoto, psio, plt, ax, thetadesired, Rxyprev, powerflag, landsum, vpused, switchflag, vpswitch, chflag, alphatdot, alphatddot, alphatdotprev, vtdot, tstart, tlast, vpactual, alphapactual, gammaactual,psi0
			###     Positions in global frame  Receive in NED, change to ENU for gazebo.
			if flag==True:
				qposarray = np.array([[xq],[yq],[zq]]);
				rposarray = np.array([[xq],[yq],[zq]]);
			xq = self.quad.pose.pose.position.y + ( 0 )			#Please replace the term in brackets with the number in gazebo just before takeoff. During slipping, the imu doesn't note these readings for some reason.
			yq = -self.quad.pose.pose.position.x + ( 0 )           #Please replace the term in brackets with the number in gazebo just before takeoff. During slipping, the imu doesn't note these readings for some reason.
			zq = self.quad.pose.pose.position.z
			xr = self.rover.pose.pose.position.y + ( 20 )
			yr = -self.rover.pose.pose.position.x + ( 0 )
			zr = self.rover.pose.pose.position.z;
			zr = 0.393242; #0.43582
			vrx = self.rover.twist.twist.linear.y;
			vry = -self.rover.twist.twist.linear.x;
			vrz = 0;
			wrz = self.rover.twist.twist.angular.z;

			vel_rover= sqrt(vrx**2+vry**2);
			exceedflag = " followed";

			numreq = 0; #if averaging is required, increase this number
			if counter<numreq:
				averagevel = averagevel + np.array([[vrx],[vry],[vrz]]);
				counter=counter+1;
			elif counter>=numreq:
				averagevel=np.array([[0],[0],[0]]);
				counter=0;
				a1 = self.quad.pose.pose.orientation.x
				b1 = self.quad.pose.pose.orientation.y
				c1 = self.quad.pose.pose.orientation.z
				d1 = self.quad.pose.pose.orientation.w

				vqx = self.quad.twist.twist.linear.y;
				vqy = -self.quad.twist.twist.linear.x;
				vqz = self.quad.twist.twist.linear.z;


				vel_rover= sqrt(vrx**2+vry**2);


				roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])
				temp = copy.deepcopy(roll);
				roll = copy.deepcopy(pitch);
				pitch = -copy.deepcopy(temp);
				yaw = yaw-pi/2;
				yaw = atan2(sin(yaw),cos(yaw))

				a2 = self.rover.pose.pose.orientation.x
				b2 = self.rover.pose.pose.orientation.y
				c2 = self.rover.pose.pose.orientation.z
				d2 = self.rover.pose.pose.orientation.w

				rollr, pitchr, yawr = tf.transformations.euler_from_quaternion([a2,b2,c2,d2])
				temp = copy.deepcopy(rollr);
				rollr = copy.deepcopy(pitchr);
				pitchr = -copy.deepcopy(temp);
				yawr = yawr-pi/2;
				yawr = atan2(sin(yawr),cos(yawr))

				if vel_rover >= 0.1:
					yawr = atan2(vry,vrx);
				else : 
					yawr = 0;

				tnow = datetime.datetime.now();

				Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch),-sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw),sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)],[sin(yaw)*cos(pitch),cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw),-sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)],[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]]);
				Rot_inertial_to_body = Rot_body_to_inertial.transpose();
				errpos[0,0] = xr-xq;
				errpos[1,0] = yr-yq;
				errpos[2,0] = zr + hoverdist - zq;
				acterrz = copy.deepcopy(zr-zq);

				psi = atan2(errpos[1,0],errpos[0,0]);

				errpos[0,0] = errpos[0,0] - hoverdist*cos(alphat + angledesired);
				errpos[1,0] = errpos[1,0] - hoverdist*sin(alphat + angledesired);

				Rxy = sqrt((errpos[0,0])**2 + (errpos[1,0])**2);
				Rz = errpos[2,0]

				R = sqrt(Rxy**2+Rz**2);
				alphat = yawr;
				alphatdotprev = alphatdot;
				if alphatdotprev!=wrz:
					deltaalphatdot = tnow-tstartalphatdot;
					deltaalphatdot = deltaalphatdot.total_seconds();
					tstartalphatdot=tnow;
					alphatddot = (wrz - alphatdotprev)/deltaalphatdot;
				alphatdot = wrz;
				if vel_rover!=vt:
					deltavt = tstartvt-tnow;
					deltavt = deltavt.total_seconds();
					tstartvt=tnow;
					vtprev = vt;
					vtdot = (vel_rover - vtprev)/deltavt;
				vt = vel_rover;
				

				vpmax = 8;


				if Rxy<7.5 and not changeflag:
					acc="normal";
					changeflag=True;
					flag3=False;
				elif Rxy>30 and changeflag:
					acc="highmaneuvering";
					changeflag=False;
					flag3=False;


				if Rxy<0.5*Rxyo or Rxy>Rxyo + 5:
					flag3=False;


				theta = atan(-Rz/Rxy);
				if flag==False:
					vp = 5;
					flag=True;

				vpactual = sqrt(vqx**2+vqy**2 + vqz**2);
				alphapactual = atan2(vqy,vqx);
				if vpactual>0:
					gammaactual = asin(vqz/vpactual);

				try:
					Rdotxy = vt*cos(alphat-psi) - vp*cos(gamma)*cos(alphap-psi);
				except:
					print("error here");
					print(" vp alphap gamma %s" , np.array([[vp],[alphap],[gamma]]))
					print(cos(gamma)*cos(alphap-psi));
				psidot = (vt*sin(alphat-psi) - vp*cos(gamma)*sin(alphap-psi))/Rxy;
				Rdotz = -vp*sin(gamma);
				thetadot = -(vp*sin(gamma)*cos(theta)-Rdotxy*sin(theta))/R;

				
				deltat = tnow-tnow_p;
				deltat_p = tnow_p - tnow_p_p;
				deltat = deltat.total_seconds();
				deltat_p = deltat_p.total_seconds();


				if flag4== False:
					psi0 = psi;
					flag4 = False;


				if flag3 == False:  # This is to store inital values
					Rxyo = copy.deepcopy(Rxy)
					alphato = copy.deepcopy(alphat)
					Rzo = copy.deepcopy(Rz);
					psio = copy.deepcopy(psi)
					vpo = copy.deepcopy(vp)
					vto = copy.deepcopy(vt)
					Rdotxyo = copy.deepcopy(Rdotxy)
					psidoto = copy.deepcopy((vt*sin(alphato-psio) - vp*cos(gamma)*sin(alphap-psio))/Rxy)
					alphatdoto = copy.deepcopy(alphatdot)
					flag3 = True;
					flag2=False;

				angled = psi0; 
				#Controller Here----------------------------------------------------------------------------------------------------------------------------------------------

				ka = 0.2;
				kc = ka;
				ka2=ka;

				varforS30 = psio-(alphato + angledesired);
				varforS30 = self.wrapangle(varforS30);
				if acc=="highmaneuvering":
					varforS30 = self.wrapangle(psio-angled);
				
				
				if powerflag==0:
					if Rxyo<=2:
						ka2 = abs(2.0*(4./6.)*((np.tanh(2)/np.tanh(1+vto))**0.8)*(3.0**(-2.75*abs(varforS30)/pi))*2**(-2.5*abs(Rzo+tan(thetadesired)*Rxyo)/Rxyo)/Rxyo**0.65);

					elif Rxyo <=7.5:	
						ka2 = abs(2.25*(4./6.)*((np.tanh(2)/np.tanh(1+vto))**0.8)*(3.0**(-2.75*abs(varforS30)/pi))*2**(-2.5*abs(Rzo+tan(thetadesired)*Rxyo)/Rxyo)/Rxyo**0.65);

					elif Rxyo <=15:
						ka2 = abs(1.35*(4./6.)*((np.tanh(2)/np.tanh(1+vto))**0.8)*(3.0**(-2.5*abs(varforS30)/pi))*2**(-2.25*abs(Rzo+tan(thetadesired)*Rxyo)/Rxyo)/Rxyo**0.65);

					elif Rxyo <= 35:
						ka2 = abs(3.5*(4./6.)*((np.tanh(2)/np.tanh(1+vto))**0.8)*(3.0**(-2.75*abs(varforS30)/pi))*2**(-2.0*abs(Rzo+tan(thetadesired)*Rxyo)/Rxyo)/Rxyo);

					elif Rxyo <=60:
						ka2 = abs(2.85*(4./6.)*((np.tanh(2)/np.tanh(1+vto))**0.8)*(3.0**(-2.25*abs(varforS30)/pi))*2**(-2.0*abs(Rzo+tan(thetadesired)*Rxyo)/Rxyo)/Rxyo);

					else:
						ka2 = abs(2.85*(4./6.)*((np.tanh(2)/np.tanh(1+vto))**0.8)*(3.0**(-2.25*abs(varforS30)/pi))*2**(-2.0*abs(Rzo+tan(thetadesired)*Rxyo)/Rxyo)/Rxyo);

				ka = ka2;
				kb = 2*ka*2**(-0.863*abs(varforS30)/pi); #0.736
				if powerflag:
					kc= 1.1*(np.sign((Rzo+tan(thetadesired)*Rxyo))*abs(Rzo+tan(thetadesired)*Rxyo)**(2.0/3.0))/(Rxyo**(2.0/3.0))*ka;
				else:
					kc=3*ka*2**(-0.536*abs(2*(Rzo+tan(thetadesired)*Rxyo)/Rxyo));

				varforS3 = psi-(alphat + angledesired);
				varforS3 = self.wrapangle(varforS3);	
				if acc=="highmaneuvering":
					varforS3 = self.wrapangle(psi - angled);

				if Rxy < 5:
					kb = 2*ka*2**(0.736*abs(varforS30)/pi);
					kc = 3*ka*2**(0.636*abs(2*(Rzo+tan(thetadesired)*Rxyo)/Rxyo));
					if abs(varforS3)<abs(varforS30)/2 or abs(varforS3)>abs(varforS30)+pi/4:
						flag3=False;
					if abs(Rz + tan(thetadesired)*Rxy) < abs(Rzo + tan(thetadesired)*Rxyo)/2 or abs(Rz + tan(thetadesired)*Rxy) > abs(Rzo+tan(thetadesired)*Rxyo) + 5:
						flag3=False;
					if Rxyprev>=5 and Rxy<5:
						flag3=False;
				if Rxyprev<5 and Rxy >= 5:
					flag3=False;


				if flag2 == False:
					if acc=="normal":
						So = np.array([[Rdotxy + ka*Rxy*abs(Rxy)**(-2.0*powerflag/3.0)],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)*abs(Rz+tan(thetadesired)*Rxy)**(-2.0*powerflag/3.0)], [psidot - alphatdot + kb*(self.wrapangle(psi - alphat- angledesired))*abs(self.wrapangle(psi - alphat- angledesired))**(-2.0*powerflag/3.0)]]);
					else:
						So = np.array([[Rdotxy + ka*Rxy*abs(Rxy)**(-2.0*powerflag/3.0)],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)*abs(Rz+tan(thetadesired)*Rxy)**(-2.0*powerflag/3.0)], [psidot + kb*(self.wrapangle(psi - angled))*abs(self.wrapangle(psi - angled))**(-2.0*powerflag/3.0)]]);
					flag2 = True;
				degree = 5.0;
				degreenum=3.0;
				constused = 1;
				if acc!="highmaneuvering":
					forceterm = alphatdot-kb/constused*(self.wrapangle(psi-(alphat+angledesired)));
				else:
					forceterm = -kb/constused*(self.wrapangle(psi-angled));
				
				k1=min([max([1.5*degree/(degree-degreenum)*(abs(So[0,0]))**((degree-degreenum)/degree)*max([abs(Rdotxyo),ka*(Rxyo)*abs(Rxyo)**(-2.0*powerflag/3.0)])/Rxyo,0,1/100*degree/(degree-degreenum)*((abs(So[0,0])**(degree-degreenum))**(1.0/degree)),0.5]),2]);
				k2=min([max([((abs(So[1,0])**(degree-degreenum)/abs(So[0,0])**(degree-degreenum))**(1.0/float(degree))*k1),0.5]),2]);
				k3=min([max([(((abs(float(So[2,0]))**(degree-degreenum)/abs(So[0,0])**(degree-degreenum))**(1.0/float(degree)))*k1),0.5]),2]);

				kswitch = 0.3*exp(1-vel_rover**0.2);
				theta = atan(-Rz/Rxy);

				if switchcounter <0:   ## an inital velocity has to be given, otherwise the controller fails for zero pursuer velocity. Change this section as you wish
					if switchcounter <0:
						vp = 5;
						alphap = pi;
						gamma = 0;
					else:
						vp = sqrt((tan(thetadesired)*Rdotxy + kc/constused*(Rz+tan(thetadesired)*Rxy))**2+(vt*cos(alphat-psi)+ka/constused*Rxy)**2+(vt*sin(alphat-psi)-forceterm*Rxy)**2);
						gamma = asin((tan(thetadesired)*Rdotxy+kc/constused*(Rz+tan(thetadesired)*Rxy))/vp);						
						alphap = self.wrapangle(psi + atan2(vt*sin(alphap-psi)-forceterm*Rxy,vt*cos(alphat-psi)+ka/constused*Rxy));
					switchcounter=switchcounter+1;
					printmode = "forcing sliding surface";

				else:					## Main code logic
					rk4 = True;
					y = np.array([alphat,vt,vp,gamma,alphap,Rxy,Rz,psi]);
					y1 = self.giveinputs(y,angledesired,ka,kb,kc,k1,k2,k3,alphatdot,alphatddot,vtdot, angled);
					y2 = self.giveinputs(y+y1*deltat/2,angledesired,ka,kb,kc,k1,k2,k3,alphatdot,alphatddot,vtdot, angled);
					y3 = self.giveinputs(y+y2*deltat/2,angledesired,ka,kb,kc,k1,k2,k3,alphatdot, alphatddot,vtdot, angled);
					y4 = self.giveinputs(y+y3*deltat,angledesired,ka,kb,kc,k1,k2,k3,alphatdot,alphatddot,vtdot, angled);

					if rk4:
						ynext = y+ (y1/6+y2/3+y3/3+y4/6)*deltat;
					else:
						ynext = y+y1*deltat;

					vp = ynext[2];
					alphap = ynext[4];
					gamma = ynext[3];

					vpdot = y1[2];
					alphapdot = y1[4];
					gammadot = y1[3];

					try :
						if powerflag:
							S = np.array([[Rdotxy + ka*np.sign(Rxy)*abs(Rxy)**(1./3.)],[Rdotz+tan(thetadesired)*Rdotxy + kc*np.sign(Rz+tan(thetadesired)*Rxy)*abs(Rz+tan(thetadesired)*Rxy)**(1./3.)], [psidot - alphatdot + kb*np.sign(varforS3)*abs(varforS3)**(1./3.)]]);
						else:
							S = np.array([[Rdotxy + ka*Rxy],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)], [psidot - alphatdot + kb*(varforS3)]]);
						if acc=="highmaneuvering":
							varforS3 = self.wrapangle(psi-angled);
							if powerflag:
								S = np.array([[Rdotxy + ka*np.sign(Rxy)*abs(Rxy)**(1./3.)],[Rdotz+tan(thetadesired)*Rdotxy + kc*np.sign(Rz+tan(thetadesired)*Rxy)*abs(Rz+tan(thetadesired)*Rxy)**(1./3.)], [psidot + kb*np.sign(varforS3)*abs(varforS3)**(1./3.)]])
							else:
								S = np.array([[Rdotxy + ka*Rxy],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)], [psidot + kb*(varforS3)]])
					except:
						raise Exception('not working!');


					vpdotprev = copy.deepcopy(vpdot);
					vtprev = copy.deepcopy(vt);
					alphapdotprev = copy.deepcopy(alphapdot);
					gammadotprev = gammadot;

					f.write("\nControl vector : \n %s \n" %(np.array([[y1[2]],[y1[4]],[y1[3]]])));
					f.write("\nSliding surface : \n %s \n" %(S));
					if landflag==False:
						rospy.loginfo("\nControl vector : \n %s \n",(np.array([[y1[2]],[y1[4]],[y1[3]]])));
						rospy.loginfo("\nSliding surface : \n %s \n", S);

					if acc=="normal":
						printmode = 'small dist';
					else:
						printmode ="large dist";

				f.write("\n vtdot, alphat, alphatdot, alphatddot: \n %s \n" %(np.array([vtdot, alphat, alphatdot, alphatddot])));
				rospy.loginfo("\n vtdot, alphat, alphatdot, alphatddot: \n %s \n", (np.array([vtdot, alphat, alphatdot, alphatddot])));
				f.write("\nsampling time : \n %s \n" %(deltat));
				rospy.loginfo("\n sampling time : \n %s \n", deltat);

				alphatdprev = alphatprev;
				alphatprev = alphat;

				if vp<0:
					vp = abs(vp);
					alphap = alphap-pi;
					gamma = pi/2-gamma;

				if vp> vpmax:
					vpused = vpmax;
					if vp>30:
						exceedflag = " exceeded!";
				else:
					vpused = vp;
					exceedflag = " followed";
				# vp=vpused;
				if abs(gamma) > 3*pi/8 and False:
					gammaused = np.sign(gamma)*3*pi/8;
					gamma = copy.deepcopy(gammaused);
				else:
					gammaused=gamma;

				vpswitch = vpmax

				alphap = self.wrapangle(alphap);
				gamma = self.wrapangle(gamma);

				Rxyprev=Rxy;

				#--------------------------------------------------------------------------------------------controller done-----------------------------------------------------------------------------------------------

				velpursuer[0] = vpused*cos(gammaused)*cos(alphap);
				velpursuer[1] = vpused*cos(gammaused)*sin(alphap);
				velpursuer[2] = vpused*sin(gammaused);

				if landflag and acterrz>=-0.01:
					landsum=landsum+1;
					if landsum >= 5:
						reached=True;

				if Rxy<0.5 and acterrz<0 and acterrz>-1:
					landflag=True
					if Rxy<0.3:
						print('\n\nreached level\n\n');
						printflag = True;
						self.land()


				   ## Msg has to be in mavros coordinate frame
				if landflag!=True:
					msg.linear.x = -velpursuer[1]
					msg.linear.y = velpursuer[0]
					msg.linear.z = velpursuer[2]
					a = (pi/2)/(pi/3)**(log(4)/log(6)); # 4.5(!4)
					msg.angular.z = psidot + a*(np.sign(self.wrapangle(psi-yaw))*abs(self.wrapangle(psi-yaw))**(log(4)/log(6)));
					if printsflag!=True:
						printflag=False;
				else:
					msg.linear.x = -velpursuer[1];
					msg.linear.y = velpursuer[0];
					a = (pi/2)/(pi/3)**(log(4)/log(6));#4.5 (!4)
					msg.angular.z = psidot + a*(np.sign(self.wrapangle(psi-yaw))*abs(self.wrapangle(psi-yaw))**(log(4)/log(6)));
				if abs(msg.angular.z) > 90*pi/180:
					msg.angular.z = np.sign(msg.angular.z)*90*pi/180;	
				pub.publish(msg);
				tnow_p_p = copy.deepcopy(tnow_p);
				tnow_p = tnow;
				printsflag = False
				if landflag:
					printmode = 'landing';
				if reached==True:
					printmode = "landed!!!";
				if printflag != True and printsflag!=True and reached!=True:
					if landflag== False:
						rospy.loginfo("rover vel observed are \n %s \n\n  ", vel_rover);
						rospy.loginfo("velocities observed\n %s\n\n", np.array([[vqx],[vqy],[vqz]]));
						rospy.loginfo("velocity published :\n %s\n\n", velpursuer);
						rospy.loginfo("\n tuning params : \n %s \n", [ka,kc,kb,k1,k2,k3]);
						rospy.loginfo("\n vp vpused alphap gamma, gammaused: \n %s \n", np.array([vp, vpused, alphap,gamma, gammaused]));
						rospy.loginfo("\n force terms vp, alphap, gamma: \n %s \n", np.array([vpFORCE, alphapFORCE,gammaFORCE]));
					# rospy.loginfo("distances,Rxyo,alphatdot \n %s \n\n", [Rxy,Rz,R,Rxyo,alphatdot]);
					rospy.loginfo("initials [Rxyo,Rdotxyo,vpo,vto,alphatdoto,psio,psidoto, varforS30] :\n %s \n\n", [Rxyo,Rdotxyo,vpo,vto,alphatdoto,psio,psidoto,varforS30]);
					rospy.loginfo("\n yaw and los diff; angular rate given; angular observed \n %s \n" , np.array([self.wrapangle(psi-yaw), msg.angular.z, wrz]));
					rospy.loginfo("\n required objectives [Rxy, theta, angledesired, angled] : \n %s \n", np.array([Rxy, Rz+tan(thetadesired)*Rxy,varforS3]));
					rospy.loginfo("\n mode : %s  , acc: %s \n\n\n" ,printmode, acc);

def listener():
	global fig , vp , alphap, gamma, plt, ax
	server = Server()
	rospy.Subscriber("/rover/local_position/odom", Odometry, server.rover_callback)
	rospy.Subscriber("/quad/local_position/odom", Odometry, server.quad_callback)
	timer=rospy.Timer(rospy.Duration(1.0/30.0), server.compute_stuff)
	rospy.spin()


rospy.init_node('purepursuit', anonymous=True)
pub = rospy.Publisher("/quad/setpoint_velocity/cmd_vel_unstamped", Twist , queue_size=10)
msg = Twist()


if __name__ == '__main__':
	try:
		global errpos, flag, flag3, changeflag, Rzo, Ro, printflag, qposarray, rposarray, fig, printsflag, landflag, velpursuer, vt, yawr, vel_rover, hoverdist, reached, averagevel, counter, flag2, vp, alphap, gamma, vpmax, Rdotxyo, switchcounter,tnow,tnow_p,tnow_p_p, angledesired,So, S,A,B, vpdotprev, alphapdotprev, gammadotprev, alphat, alphatprev, alphatdprev, alphatdot, alphapdotprev, deltat, deltat_p, vtprev, acc, alphatdoto, psidoto, psio, fig, ax, thetadesired, Rxyprev, powerflag, landsum, vpused, chflag, tstartalphatdot, alphatddot, tstartvt, vtdot, tstart, tlast, vpactual, alphapactual, gammaactual, psi0
		reached = False;
		averagevel = np.array([[0],[0],[0]]);
		counter = 0;
		hoverdist = 0.2; ## reach here before landing
		vel_rover=0;
		vt=1;
		yawr=0;
		thetadesired = pi/4;
		velpursuer = np.array([[0.0],[0.0],[0.0]]);
		printsflag = False
		landflag = False
		qposarray = np.zeros((3,1));
		rposarray = np.zeros((3,1));
		flag2 = False;
		flag3 = False;
		changeflag=False;
		vpmax = min([max([5,2*vt]),8]);  # change these numbers to expected upper values of vpmax
		switchcounter=-10;
		alphap=0;
		gamma=0;
		tnow_p_p=datetime.datetime.now();
		time.sleep(0.033);
		tnow_p=datetime.datetime.now();
		time.sleep(0.033);
		tnow = datetime.datetime.now();
		So = np.zeros((3,1));
		S = np.zeros((3,1));
		A=np.zeros((3,3));
		B = np.zeros((3,1));
		vpdotprev =0;
		alphapdotprev=0;
		gammadotprev=0;
		alphatprev = 0;
		alphat = 0;
		alphatdprev=0;
		alphatdot = 0;
		alphatdotprev = 0;
		deltat = 0.033;
		deltat_p = 0.033;
		tstartalphatdot=tnow_p;
		tstartvt=tnow_p;
		tstart= datetime.datetime.now();
		vtprev = 0;
		acc="highmaneuvering";
		powerflag=0;
		landsum = 0;
		vpused=5;
		vp = 5;
		switchflag=0;
		chflag= True;
		Rxyprev = 0;
		Rxyo=0;
		vtdot=0;
		alphatddot =0;
		tlast=0;
		vpactual=0;
		alphapactual=0;
		gammaactual=0;

		angledesired = pi/2;  ## This is desired approach angle in x-y plane

		errpos = np.zeros((3,1));
		flag= False;
		# Rxyo =0;
		Rzo = 0;
		Ro = 0;
		printflag= False;
		f = open("logdata.txt","a+");
		f.write('*\n*\n*\n*===============================================================================================================================================================================================================================================================STARTS HERE============================================================================================================================================================================================================================================================================================================================================================================================================================================================*\n\n');
		f.close()
		f2 = open("two_bot_pos.txt","w");
		f2.write("");
		f2.close();
		listener();
	except rospy.ROSInterruptException:
		pass
