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
from rosgraph_msgs.msg import Clock
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

vision = False;
simtime = None
turndir = 0;

class Server:
	def __init__(self):
		self.quad = Odometry();
		if not vision:
			self.rover = Odometry();
		else:
			self.rover = kalman()

		self.timing = Clock();

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

		# Compute stuff.
		# self.compute_stuff()

	def quad_callback(self, dat):
		# "Store" the message received.
		self.quad = dat;
	def time_callback(self, dat):
		# "Store" the message received.
		self.timing = dat.clock.secs + dat.clock.nsecs*10**(-9);

		# Compute stuff.
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

		k20=0.5
		k21 = 0.5/(Rxy+0.01)

		S = np.array([[Rdotxy + ka*Rxy], [Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)], [psidot - alphatdot + kb*varforS3]]);

		if powerflag:
			S = np.array([[Rdotxy + ka*np.sign(Rxy)*abs(Rxy)**(1.0/3.0)],[Rdotz+tan(thetadesired)*Rdotxy + kc*np.sign(Rz+tan(thetadesired)*Rxy)*abs(Rz+tan(thetadesired)*Rxy)**(1.0/3.0)], [psidot - alphatdot + kb*np.sign(varforS3)*abs(varforS3)**(1.0/3.0)]]);
		else:
			S = np.array([[Rdotxy + ka*Rxy],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)], [psidot - alphatdot + kb*(varforS3)]]);
		if acc=="highmaneuvering":
			varforS3 = self.wrapangle(psi-angled);
			if powerflag:
				S = np.array([[Rdotxy + ka*np.sign(Rxy)*abs(Rxy)**(1.0/3.0)],[Rdotz+tan(thetadesired)*Rdotxy + kc*np.sign(Rz+tan(thetadesired)*Rxy)*abs(Rz+tan(thetadesired)*Rxy)**(1./3.)], [psidot + kb*np.sign(varforS3)*abs(varforS3)**(1./3.)]])
			else:
				S = np.array([[Rdotxy + ka*Rxy],[Rdotz+tan(thetadesired)*Rdotxy + kc*(Rz+tan(thetadesired)*Rxy)], [psidot + kb*(varforS3)]])
		
		A = np.array([[-cos(alphap-psi)*cos(gamma), vp*sin(alphap-psi)*cos(gamma), vp*cos(alphap-psi)*sin(gamma)],[-sin(gamma), 0, -vp*cos(gamma)],[-sin(alphap-psi)*cos(gamma), -vp*cos(alphap-psi)*cos(gamma), vp*sin(alphap-psi)*sin(gamma)]]);
		A = np.matmul(np.array([[1, 0, 0],[tan(thetadesired), 1, 0],[0, 0, 1]]), A);

		term2=(Rz+tan(thetadesired)*Rxy);
		term1 = Rxy;
		term3 = varforS3;
		if powerflag:
			if abs(term1)<0.01:
				term1=0.01*np.sign(term1);
			if abs(term2)<0.01:
				term2=0.01*np.sign(term2);
			if abs(term3)<0.01:
				term3=0.01*np.sign(term3);
		if powerflag:
			angleterm = -kb*Rxy*(psidot-alphatdot)*(np.sign(term3)*(1.0/3.0*abs(term3)**(-2.0/3.0)))**powerflag;
		else:
			angleterm = -kb*Rxy*(psidot-alphatdot);
		if acc=="highmaneuvering":
			angleterm = -kb*Rxy*(psidot)*(np.sign(term3)*(1.0/3.0*abs(term3)**(-2.0/3.0)))**powerflag;
		b1=-k20*np.sign(S[0,0])-k1*np.sign(S[0,0])*abs(S[0,0])**(degreenum/degree) + vp*sin(alphap-psi)*cos(gamma)*psidot - vtdot*cos(alphat-psi)+vt*sin(alphat-psi)*(alphatdot-psidot) - ka*(Rdotxy)*(np.sign(term1)*(1.0/3.0*abs(term1)**(-2.0/3.0)))**powerflag;
		b2=-k20*np.sign(S[1,0])-k2*np.sign(S[1,0])*abs(S[1,0])**(degreenum/degree) + tan(thetadesired)*(vp*sin(alphap-psi)*cos(gamma)*psidot - vtdot*cos(alphat-psi)+vt*sin(alphat-psi)*(alphatdot-psidot)) - kc*(Rdotz + tan(thetadesired)*Rdotxy)*((1.0/3.0*np.sign(term2)*abs(term2)**(-2.0/3.0)))**powerflag;
		b3=-k21*Rxy*np.sign(S[2,0])-Rxy*k3*np.sign(S[2,0])*abs(S[2,0])**(degreenum/degree) +angleterm + alphatddot*Rxy + Rdotxy*psidot - vp*cos(alphap-psi)*cos(gamma)*psidot - vt*cos(alphat-psi)*(alphatdot-psidot) - vtdot*sin(alphat-psi);
		if acc=="highmaneuvering":
			b3=-k21*Rxy*np.sign(S[2,0])-Rxy*k3*np.sign(S[2,0])*abs(S[2,0])**(degreenum/degree) +angleterm + Rdotxy*psidot - vp*cos(alphap-psi)*cos(gamma)*psidot - vt*cos(alphat-psi)*(alphatdot-psidot) - vtdot*sin(alphat-psi);
		B[0,0] = b1;
		B[1,0] = b2;
		B[2,0] = b3;
		# if vp >0.1 and (pi/2 - abs(gamma)) > 0.1 and ( vp>0.8 or Rxy<2 ):
		# print(np.shape(invA))
		inputvec = np.matmul(np.linalg.inv(A),B);
		vpvar = inputvec[0,0];
		alphapvar = inputvec[1,0];
		gammavar = inputvec[2,0];
		# else:
		if vp<0.1 and vpvar<0:
			vpvar=0;
		if (cos(gamma))<0.1 and np.sign(gammavar*gamma)>0:
			gammavar= 0
		vpdot = vpvar;
		alphapdot= alphapvar;
		gammadot = gammavar;

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
			global alphato, xq, yq, zq, xr, yr, zr, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, tnow, tnow_p, tnow_p_p, error_head_prev, goal, goal_body, errpos, flag, flag3, changeflag, Rxyo, Rzo, Ro, printflag, qposarray, rposarray, fig, printsflag, landflag, velpursuer, alphap,vt, yawr, hoverdist, reached, averagevel, counter, flag2, vp, gamma, alphat, Rdotxyo, vpmax, switchcounter,tnow, angledesired, k1, k2, k3, ka, kb, S,So, A,B, vpdotprev, alphapdotprev, gammadotprev, alphatprev,alphatdprev, deltat, deltat_p , tstartalphatdot, tstartvt, vt,vtprev, Rxy, Rz, vpo, vto, acc, alphatdoto, psidoto, psio, plt, ax, thetadesired, Rxyprev, powerflag, landsum, vpused, switchflag, vpswitch, chflag, alphatdot, alphatddot, alphatdotprev, vtdot, tstart, tlast, vpactual, alphapactual, gammaactual, psi0, simtime, turnflag, turndir
			###     Positions in global frame  Receive in NED, change to ENU for gazebo.
			if flag==True:
				qposarray = np.array([[xq],[yq],[zq]]);
				rposarray = np.array([[xq],[yq],[zq]]);
			xq = self.quad.pose.pose.position.y + ( 0 )			#Please replace the term in brackets with the number in gazebo just before takeoff. During slipping, the imu doesn't note these readings for some reason.
			yq = -self.quad.pose.pose.position.x + ( 0 )           #Please replace the term in brackets with the number in gazebo just before takeoff. During slipping, the imu doesn't note these readings for some reason.
			zq = self.quad.pose.pose.position.z
			if vision == True:
				xr = self.rover.pos.x;  #Change the sources of these
				yr=self.rover.pos.y;	 #implementing rangekutta
				zr= self.rover.pos.z;	 #
				vrx = self.rover.speed*np.cos(self.rover.theta);	 #
				vry = self.rover.speed*np.sin(self.rover.theta);	 #
				vrz = 0;
				wrz = self.rover.omegaz
			else:
				xr = self.rover.pose.pose.position.y + (20)#( 86.6025403784 )
				yr = -self.rover.pose.pose.position.x + (0)#( -50 )
				zr = self.rover.pose.pose.position.z;
				zr = 0.393242; #0.43582
				vrx = self.rover.twist.twist.linear.y;
				vry = -self.rover.twist.twist.linear.x;
				vrz = 0;
				wrz = self.rover.twist.twist.angular.z;

			vel_rover= sqrt(vrx**2+vry**2);
			exceedflag = " followed";

			numreq = 0; #if averaging is required, increase this number
			# switchcounter = -10;
			if counter<numreq:
				averagevel = averagevel + np.array([[vrx],[vry],[vrz]]);
				counter=counter+1;
			elif counter>=numreq:
				# averagevel = averagevel/numreq;

				# vrx = averagevel[0,0];
				# vry = averagevel[1,0];

				averagevel=np.array([[0],[0],[0]]);
				counter=0;
				# switchcounter = -10
				###     Orientations in global frame invert Y and Z axes for orientation and accel for NED to ENU
				a1 = self.quad.pose.pose.orientation.x
				b1 = self.quad.pose.pose.orientation.y
				c1 = self.quad.pose.pose.orientation.z
				d1 = self.quad.pose.pose.orientation.w
				vqx = self.quad.twist.twist.linear.y;
				vqy = -self.quad.twist.twist.linear.x;
				vqz = self.quad.twist.twist.linear.z;
				vel_rover= sqrt(vrx**2+vry**2);
				q_new = tf.transformations.quaternion_multiply([0,0,-sin(np.pi/4),cos(np.pi/4)],[a1,b1,c1,d1])
				# q_new = tf.transformations.quaternion_multiply(q_new,[0,0,-sin(np.pi/4),cos(np.pi/4)])
				a1 = q_new[0];
				b1 = q_new[1];
				c1 = q_new[2];
				d1 = q_new[3];

				roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

				yaw = atan2(sin(yaw),cos(yaw))
				if vision!=True:
					a2 = self.rover.pose.pose.orientation.x
					b2 = self.rover.pose.pose.orientation.y
					c2 = self.rover.pose.pose.orientation.z
					d2 = self.rover.pose.pose.orientation.w
					q_new = tf.transformations.quaternion_multiply([0,0,-sin(np.pi/4),cos(np.pi/4)],[a2,b2,c2,d2])
					# q_new = tf.transformations.quaternion_multiply(q_new,[0,0,-sin(np.pi/4),cos(np.pi/4)])
					a2 = q_new[0];
					b2 = q_new[1];
					c2 = q_new[2];
					d2 = q_new[3];
					rollr, pitchr, yawr = tf.transformations.euler_from_quaternion([a2,b2,c2,d2])
					yawr = atan2(sin(yawr),cos(yawr))
				else : 
					yawr = self.rover.theta;
					if self.rover.detected:
						vel_rover = self.rover.speed;
					else:
						vel_rover = self.rover.speed;
				while not self.timing:
					time.sleep(0.25);
				simtime = self.timing
				tnow = simtime;
				if not tstart:
					tstart = simtime;
				
				Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch),-sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw),sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)],[sin(yaw)*cos(pitch),cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw),-sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)],[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]]);
				Rot_inertial_to_body = Rot_body_to_inertial.transpose();
				errpos[0,0] = xr-xq;
				errpos[1,0] = yr-yq;
				errpos[2,0] = zr + hoverdist - zq; #0.142719*2 + 0.3 - zq;
				acterrz = copy.deepcopy(zr-zq);

				psi = atan2(errpos[1,0],errpos[0,0]);

				errpos[0,0] = errpos[0,0] - hoverdist*cos(alphat + angledesired);
				errpos[1,0] = errpos[1,0] - hoverdist*sin(alphat + angledesired);

				Rxy = sqrt((errpos[0,0])**2 + (errpos[1,0])**2);
				Rz = errpos[2,0]

				R = sqrt(Rxy**2+Rz**2);
				if vision!=True:
					alphat = yawr;
					alphatdotprev = alphatdot;
					if alphatdotprev!=wrz:
						deltaalphatdot = tnow-tstartalphatdot;
						# deltaalphatdot = deltaalphatdot.total_seconds();
						tstartalphatdot=tnow;
						alphatddot = (wrz - alphatdotprev)/deltaalphatdot;
					alphatdot = wrz;#(alphat - alphatprev)/deltat;
					if vel_rover!=vt:
						deltavt = tstartvt-tnow;
						deltavt = deltavt;
						tstartvt=tnow;
						vtprev = vt;
						vtdot = (vel_rover - vtprev)/deltavt;
				else:
					alphat = yawr;
					vtdot = self.rover.acc
					alphatdot = wrz;
					alphatddot = self.rover.alpha

				vt = vel_rover;
				

				vpmax = 8;


				if Rxy<7.5 and not changeflag:
					acc="normal";
					changeflag=True;
					flag3=False;
				elif Rxy>10 and changeflag:
					acc="highmaneuvering";
					changeflag=False;
					psi0 = psi;
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

				if not tnow_p:
					tnow_p = tnow;
				if not tnow_p_p:
					tnow_p = tnow_p;
				
				deltat = tnow-tnow_p;
				deltat_p = tnow_p - tnow_p_p;
				# deltat = deltat.total_seconds();
				# deltat_p = deltat_p.total_seconds();
				# if abs(deltat -0.01)>0.03:
				# 	deltat=0.01;


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
				ka2 = ka;

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
						
				elif powerflag==1:
					if Rxyo <=64: 
						ka2 = abs(1.5*((vpo-vto)/(vpo+vto))**1.2/Rxyo**(0.7*0.4));
					elif Rxyo <=216:
					    ka2 = abs(2.5*((vpo-vto)/(vpo+vto))**1.2/Rxyo**(0.7*0.4));
					elif Rxyo <= 729:
					    ka2 = abs(3.6*((vpo-vto)/(vpo+vto))**1.2/Rxyo**(0.7*0.4));
					else:
						ka2 = abs(4*((vpo-vto)/(vpo+vto))**1.2/Rxyo**(0.7*0.4));

				
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
				vpFORCE = sqrt((tan(thetadesired)*Rdotxy + kc/constused*(Rz+tan(thetadesired)*Rxy))**2+(vt*cos(alphat-psi)+ka/constused*Rxy)**2+(vt*sin(alphat-psi)-forceterm*Rxy)**2);
				gammaFORCE = asin((tan(thetadesired)*Rdotxy+kc/constused*(Rz+tan(thetadesired)*Rxy))/vpFORCE);						
				alphapFORCE = psi + atan2(vt*sin(alphap-psi)-forceterm*Rxy,vt*cos(alphat-psi)+ka/constused*Rxy);
				
				k1=min([max([1.5*degree/(degree-degreenum)*(abs(So[0,0]))**((degree-degreenum)/degree)*max([abs(Rdotxyo),ka*(Rxyo)*abs(Rxyo)**(-2.0*powerflag/3.0)])/Rxyo,0,1/100*degree/(degree-degreenum)*((abs(So[0,0])**(degree-degreenum))**(1.0/degree)),0.5]),2]);
				k2=min([max([((abs(So[1,0])**(degree-degreenum)/abs(So[0,0])**(degree-degreenum))**(1.0/float(degree))*k1),0.5]),2]);
				k3=min([max([(((abs(float(So[2,0]))**(degree-degreenum)/abs(So[0,0])**(degree-degreenum))**(1.0/float(degree)))*k1),0.5]),2]);
				
				kswitch = 0.3*exp(1-vel_rover**0.2);
				theta = atan(-Rz/Rxy);
				
				if abs(gamma)>pi/2:
					gamma=np.sign(gamma)*(pi/2-0.05);


				if switchcounter <0:   ## an inital velocity has to be given, otherwise the controller fails for zero pursuer velocity.
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

				if (vp<0):
					vp = abs(vp);
					alphap = alphap-pi;
					gamma = -gamma;

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


				alphap = self.wrapangle(alphap);
				gamma = self.wrapangle(gamma);


				Rxyprev=Rxy;

				#-------------------------------------------------------------------------------------------------------------------------------------------------------------

				velpursuer[0] = vpused*cos(gammaused)*cos(alphap);
				velpursuer[1] = vpused*cos(gammaused)*sin(alphap);
				velpursuer[2] = vpused*sin(gammaused);


				if landflag and acterrz>=-0.01:
					landsum=landsum+1;
					if landsum >= 5:
						reached=True;

				if Rxy<0.3 and acterrz<0 and acterrz>-1:
					landflag=True
					print('\n\nreached tracking point\n\n');
					printflag = True;
					self.land()
				if Rxy>1:
					printflag=False;

				if vision and self.rover.detected:
					turnflag=0
					turndir = np.sign(psi-yaw)
				elif vision:
					turnflag = turnflag+1;

				   ## Msg has to be in mavros coordinate frame
				if landflag!=True:
					msg.linear.x = -velpursuer[1]
					msg.linear.y = velpursuer[0]
					msg.linear.z = velpursuer[2]
					k5 = 0;
					a = (pi/2)/(pi/3)**(log(4)/log(6)); # 4.5(!4)
					# msg.angular.z = psidot + a*(np.sign(self.wrapangle(psi-yaw))*abs(self.wrapangle(psi-yaw))**(log(6)/log(9))); #0.6
					
					msg.angular.z = psidot + a*(np.sign(self.wrapangle(psi-yaw))*abs(self.wrapangle(psi-yaw))**(log(4)/log(6))) + k5*np.sign(self.wrapangle(psi-yaw)); #0.6 4.5 instead of 4
				else:
					k5 = 0;
					# msg.linear.x = -velpursuer[1];
					# msg.linear.y = velpursuer[0];
					a = (pi/2)/(pi/3)**(log(4)/log(6));#4.5 (!4)
					# msg.angular.z = psidot + a*(np.sign(self.wrapangle(psi-yaw))*abs(self.wrapangle(psi-yaw))**(log(6)/log(9))); #0.6
					
					msg.angular.z = psidot + a*(np.sign(self.wrapangle(psi-yaw))*abs(self.wrapangle(psi-yaw))**(log(4)/log(6)))+ k5*np.sign(self.wrapangle(psi-yaw)); #0.6 4.5 instead of 4
				if turnflag > 20 and vision:
					msg.angular.z = turndir*pi/4;
					
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
				if printsflag!=True and reached!=True:
					f.write("\n tuning params : \n %s \n" %([ka,kc,kb,k1,k2,k3]));
					f.write("velocities observed\n %s \n" %(np.array([[vqx],[vqy],[vqz]])));
					f.write("velocity published :\n %s\n" %(velpursuer));
					f.write("\nrover vel observed is \n %s \n " %(vel_rover));
					f.write("\ninitials [Rxyo, Rdotxyo, vpo, vto, vpmax, varforS30, alphatdoto] :\n %s \n\n" %([Rxyo,Rdotxyo,vpo,vto,vpmax, varforS30, alphatdoto]));
					f.write("\ninitials [So[0,0], So[1,0],So[2,0]] :\n %s \n\n" %(So));
					f.write("\n[flags flag, flag2 flag3, exceedflag, powerflag, landflag] : \n %s \n\n" %([flag,flag2, flag3, exceedflag,powerflag, landflag]))
					# f.write("distances,Rxyo,alphatdot \n %s \n\n" %([Rxy,Rz,R,Rxyo,alphatdot]));

					f.write("\n[Rdotxy,pursuer component,targetcomponent, required component,psidot] :\n %s \n\n" %([Rdotxy,vp*cos(gammaused)*cos(alphap-psi),vt*cos(alphat-psi),-ka*Rxy*abs(Rxy)**(-(2.0/3.0)*powerflag),psidot]));
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
				f.write("\n vp, vpused, alphap, gamma, gammaused: \n %s \n" %(np.array([vp, vpused,alphap,gamma, gammaused])));
				f.write("\n The speed limit has been %s \n" %(exceedflag));
				f.write("\n force terms vp, alphap, gamma: \n %s \n" %(np.array([vpFORCE, alphapFORCE,gammaFORCE])));
				f.write("\n alphat, psi, approach angle: \n %s \n" %(np.array([alphat,psi, self.wrapangle(psi-alphat)])));
				f.write("\n quad attitude, [pitch,roll,yaw] : \n %s \n " %(np.array([pitch, roll, yaw])));
				f.write("\n yaw and los diff; angular rate given; angular observed \n %s \n " %(np.array([self.wrapangle(psi-yaw), msg.angular.z, wrz])));
				f.write("\n required objectives [Rxy, theta, angledesired, angled] : \n %s \n" %(np.array([Rxy, Rz+tan(thetadesired)*Rxy, varforS3])));
				f.write("\n mode : %s  , acc: %s \n\n\n" %(printmode, acc));
				# mpl.style.use('seaborn');>
			tpassed=tnow-tstart;
			# tpassed=tpassed.total_seconds();
			f2 = open("two_bot_pos.txt","a+");
			if round(tpassed)>tlast + 0.25:
				tlast = tpassed;
				f2.write("%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n" %(xq,yq,zq,xr,yr,zr,tpassed,alphap,gamma,vpused,psi,theta,vt,alphat,Rxy,Rz,Rdotxy,Rdotz,S[0,0],S[1,0],S[2,0],vpactual,alphapactual,gammaactual, vp,k1,k2,k3,ka,kc,kb,varforS3,simtime));
			f2.close();
			# f3 = open("quad_pos.txt","w");
			# f3.write("%s %s %s %s %s %s %s" %(xq,yq,zq,yaw,pitch,roll,tpassed));
			# f3.close();

def listener():
	global fig , vp , alphap, gamma, plt, ax#, errpos, flag, flag3, changeflag, Rxyo, Rzo, Ro, printflag, qposarray, rposarray, fig, printsflag, landflag, velpursuer, vt, yawr, vel_rover, hoverdist, reached, averagevel, counter, flag2, vp, alphap, gamma, vpmax, Rdotxyo, switchcounter,tnow,tnow_p, angledesired,So, S,A,B, vpdotprev, alphapdotprev, gammadotprev, alphat, alphatprev, alphatdprev, deltat, vtprev, acc, alphato, alphatdoto, psidoto, psio, fig
	server = Server()
	if vision!=True:
		rospy.Subscriber("/rover/local_position/odom", Odometry, server.rover_callback)

	else:
		rospy.Subscriber("/bayes_filter", kalman , server.rover_callback)
		
		#Write your subscriber here-----<required values are:- vt, vtdot, alphat, alphatdot, alphatddot>-------------------------------------------------
	rospy.Subscriber("/quad/local_position/odom", Odometry, server.quad_callback)
	rospy.Subscriber("/quad/local_position/odom", Odometry, server.quad_callback)
	rospy.Subscriber("/clock", Clock, server.time_callback)
	timer=rospy.Timer(rospy.Duration(1.0/30.0), server.compute_stuff)
	rospy.spin()


rospy.init_node('SLMG', anonymous=True)
pub = rospy.Publisher("/quad/setpoint_velocity/cmd_vel_unstamped", Twist , queue_size=10)
publand = rospy.Publisher("/quad/setpoint_raw/attitude", AttitudeTarget , queue_size=10)
msg = Twist()
msgland = AttitudeTarget()

# camera_mount = 0.785398
# horizontal = 1.04719/2
# vertical = 1.04719/2


if __name__ == '__main__':
	try:
		global errpos, flag, flag3, changeflag, Rzo, Ro, printflag, qposarray, rposarray, fig, printsflag, landflag, velpursuer, vt, yawr, vel_rover, hoverdist, reached, averagevel, counter, flag2, vp, alphap, gamma, vpmax, Rdotxyo, switchcounter,tnow,tnow_p,tnow_p_p, angledesired,So, S,A,B, vpdotprev, alphapdotprev, gammadotprev, alphat, alphatprev, alphatdprev, alphatdot, alphapdotprev, deltat, deltat_p, vtprev, acc, alphatdoto, psidoto, psio, fig, ax, thetadesired, Rxyprev, powerflag, landsum, vpused, chflag, tstartalphatdot, alphatddot, tstartvt, vtdot, tstart, tlast, vpactual, alphapactual, gammaactual, psi0, turnflag
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
		vpmax = min([max([3,2*vt]),4]);  # change numbers 3 and 5 which are expected and upper values of vpmax
		switchcounter=-10;
		alphap=0;
		gamma=0;
		tnow_p_p=0;
		# time.sleep(0.033);
		tnow_p=0;
		# time.sleep(0.033);
		tnow = 0;
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
		deltat = 0.0;
		deltat_p = 0.0;
		tstartalphatdot=tnow_p;
		tstartvt=tnow_p;
		tstart= None;
		vtprev = 0;
		acc="highmaneuvering";
		powerflag=0;
		landsum = 0;
		vpused=5;
		vp = 5;
		switchflag=0;
		chflag= True;
		Rxyprev = 0;
		Rxyo=100;
		vtdot=0;
		alphatddot =0;
		tlast=0;
		vpactual=0;
		alphapactual=0;
		gammaactual=0;
		psio = 0;#-pi/6;
		psi0 = 0;
		turnflag = 0;

		angledesired = -3*pi/4;#pi/2;#pi;##pi/2;#

		errpos = np.zeros((3,1));
		flag= False;
		# Rxyo =0;
		# Rzo = 0;
		# Ro 
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
