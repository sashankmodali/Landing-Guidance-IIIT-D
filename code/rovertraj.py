#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import *
from std_msgs.msg import Float64   
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, HomePosition
from nav_msgs.msg import Odometry
import utm
import copy
import datetime

rospy.init_node('rovertrajectory', anonymous=True)
pub = rospy.Publisher("/rover/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
# pub = rospy.Publisher("/quad/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
# pub2 = rospy.Publisher("/rover/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)


receive = [0.0,0.0]
error_ang_sum = 0.0
error_ang_prev = 0.0
error_dist_prev = 0.0
error_dist_sum = 0.0
def fn(info):
    rospy.loginfo("IN LIS2")
    global receive
    receive[0] = info.geo.latitude
    receive[1] = info.geo.longitude
    receive = list(receive)
    receive = utm.from_latlon(receive[0],receive[1])
pos = 0
num = 0
d = []


def get_dist(x,y):
    a = utm.from_latlon(x[0],x[1])
    a = list(a)
    a[0] = a[0] - y[0]
    a[1] = a[1] - y[1]
    return a

def callback(data):
    global receive
    global num
    global go_to_posn_latlong_full
    global error_ang_sum
    global error_ang_prev
    global error_dist_prev
    global error_dist_sum
    global pos
    global yawdesired
    global flag
    global alphat
    global now, now_p, tstart
    global desflag
    global tlast, f2


    yawdesired = pi/4;


    ang1 =  data.pose.pose.orientation.x
    ang2 =  data.pose.pose.orientation.y
    ang3 =  data.pose.pose.orientation.z
    ang4 =  data.pose.pose.orientation.w
    l = [ang1,ang2,ang3,ang4]

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)
    temp = copy.deepcopy(roll);
    roll = copy.deepcopy(pitch);
    pitch = -copy.deepcopy(temp);
    yaw = yaw-pi/2;
    yaw = atan2(sin(yaw),cos(yaw));
    if flag:
        alphat=yaw;
        flag=False;

    # euler = list(euler)
    x = data.pose.pose.position.y
    y = -data.pose.pose.position.x
    z = 0.49;#data.pose.pose.position.z
    vrx = data.twist.twist.linear.y;
    vry = -data.twist.twist.linear.x;
    vrz = 0;
    wrz = data.twist.twist.angular.z;

    vel = np.zeros((3,1));
    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch),-sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw),sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)],[sin(yaw)*cos(pitch),cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw),-sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)],[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]]);
    # Rot_inertial_to_body = Rot_body_to_inertial.transpose();
    

    speed = 1;
    vx=0;
    vy=0;

    now = datetime.datetime.now();
    t = now-tstart;
    t=t.total_seconds();

    deltat = now-now_p
    deltat =deltat.total_seconds();
    speed = 3.0;

    # yawdesired=0;
    # if abs(yawdesired-yaw) > 0.02:
    #     alphatdot = 2./(4.)*np.sign(yawdesired-yaw)*(abs(yawdesired-yaw));
    # else:
    #     alphatdot=0;
    # #-----------------------------straight line------------------------------------------------------------------------------
    # yawdesired=pi/4
    ## alphat = yawdesired; #straighline fast


    # if abs(yawdesired-yaw) > 0.01 and desflag!=1:
    #     alphatdot = 2./(speed+1)*np.sign(yawdesired-yaw)*(abs(yawdesired-yaw))**(0.3333);
    # else:
    #     desflag=1;
    #     alphatdot=0;

    # if abs(yawdesired-yaw) > 0.1:
    #     desflag=0;


    ## alphatdot=0;
    #-----------------------------circular-----------------------------------------------------------------------------------
    # alphatdot = pi/12;
    #-----------------------------sinusoidal---------------------------------------------------------------------------------
    alphatdot = pi/6*sin(pi/4*t);

    #-------------------------------------------------------------------------------------------------------------------------
    # alphat = alphat + deltat*alphatdot;
    # while abs(alphat)>pi:
    #     alphat=alphat-np.sign(alphat)*2*np.pi;

    # vel = np.array([[vx],[vy],[0]]);
    vel = np.array([[speed],[0],[0]]);
    RotMat = np.identity(3);
    velglob = np.matmul(RotMat,vel);

    msg = Twist()

    msg.linear.x=-velglob[1];
    msg.linear.y=velglob[0];
    msg.linear.z=velglob[2];
    msg.angular.z = alphatdot;

    rate = rospy.Rate(10);

    now_p=now;
    pub.publish(msg);

    rospy.loginfo("position of rover is :\n %s\n\n ", [x,y,z]);
    # rospy.loginfo("velocity published :\n %s\n\n", msg);
    # rospy.loginfo("velocity observed :\n %s\n\n", data.twist.twist.linear);
    rospy.loginfo("rover speed published and observed: %s\n\n", [speed, sqrt(vrx**2 + vry**2 + vrz**2)]);
    rospy.loginfo("rover angles published and observed: %s\n\n", [alphat, atan2(vry,vrx), alphatdot, wrz]);

    rospy.loginfo("\n angles are %s \n\n", [roll, pitch, yaw]);
    rospy.loginfo("\n sampling time %s \n\n", deltat);
    rospy.loginfo("\n sim time %s \n\n", t);

    f2 = open("rovertraj.txt","a+");
    if t>tlast + 0.5:
        tlast = t;
        f2.write("%s %s %s %s %s %s %s %s %s %s\n" %(x,y,z,t,speed,sqrt(vrx**2 + vry**2 + vrz**2),alphat,atan2(vry,vrx),wrz,yaw));
    f2.close();


def listener():
    # rospy.Subscriber("/rover/home_position/home", HomePosition, fn)
    rospy.Subscriber("/rover/local_position/odom", Odometry, callback)
    # rospy.Subscriber("/quad/local_position/odom", Odometry, callback);
    rospy.spin()
    

if __name__ == '__main__':
    try:
        global flag, now, now_p,alphat, tstart, desflag, tlast, f2
        now=datetime.datetime.now();
        tstart=now;
        tlast=now-tstart;
        tlast=tlast.total_seconds();
        now_p=now;
        desflag=0;
        flag=True;
        f2 = open("rovertraj.txt","w");
        f2.write("");
        f2.close();
        listener()
    except rospy.ROSInterruptException:
        pass