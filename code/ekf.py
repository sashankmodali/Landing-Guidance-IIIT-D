#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import scipy.signal as sig
import numpy as np
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import *
# from test.msg import *
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time
import control.matlab as mb
from timeit import default_timer as timer
import timeit
import copy

rospy.init_node('bayes_filter', anonymous=True)
pub = rospy.Publisher("/bayes_filter", kalman, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0
x = 0.0
y = 0.0
z = 0.0
u1_prev = 0.0
u2_prev = 0.0
u3_prev = 0.0
x1_prev = 0.0
x2_prev = 0.0
v_roll = 0.0
v_pitch = 0.0
v_yaw = 0.0
v1_prev = 0.0
v2_prev = 0.0
i = 0
v1 = 0.0
v2 = 0.0

detect = 1

now_cam_p = timer()
now_cam = timer()
now_kal_p = timer()
now_kal = timer()

####    Vert and hor fov
vert_fov = pi/4
hori_fov = 1280.0/720.0*vert_fov                         #compute vert_fov to hori_fov


goal_pred = np.array([[0.0]
                    ,[0.0]])    #target states estimated by the camera homography

goal_pred_var = np.array([[1.0, 0]
                        ,[0, 1.0]])     #variance in the estimates by the camera homography

##      Rotational matrices
Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()


X = np.array([[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]])        #target states estimated by the kalman filter

P_k = np.eye(6);
# P = np.array([[np.random.normal(0, 1), 0, 0, 0]
#             ,[0, np.random.normal(0, 0.25), 0, 0]
#             ,[0, 0, np.random.normal(0, 1), 0]
#             ,[0, 0, 0, np.random.normal(0, 0.25)]])     #variance in the filter

F = np.array([[0,0,0,0],[0,0,0,0],[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]);

# P = F @ P @ F.T;



tprev=0;

msg = kalman()

xr=0
yr=0
zr=0
yawr=0
v_yaw_r=0
v_xr=0
v_yr=0

include_vel_meas = True;
get_vel_flag = True;


def kalmanfunc(timer):
    global now_kal, now_kal_p, X, P_k, v_x, v_y, v_z, x, y, z, goal_pred, goal_pred_var, detect, xr,yr,zr,yawr,v_yaw_r,v_xr,v_yr,yaw,v_yaw,pitch,roll,simtime,get_vel_flag
    
    del_t = 0.01
    tnow = 0;

    ####    Apply kalman filter
    theta = float(X[2,0])
    om = float(X[3,0])
    v = float(X[4,0])
    a = float(X[5,0])
    A = np.array([[1, 0, -v*sin(theta)*del_t, 0, cos(theta)*del_t, 0],[0, 1, v*cos(theta)*del_t, 0, sin(theta)*del_t, 0],[0, 0, 1, del_t, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, del_t ],[0, 0, 0, 0, 0, 1]])


    #prediction of the next state
    X_new_pred = np.array([[float(X[0,0]) + v*cos(theta)*del_t + 0.5*a*cos(theta)*del_t**2],[float(X[1,0]) + v*sin(theta)*del_t + 0.5*a*sin(theta)*del_t**2],[theta + om*del_t],[om],[v + a*del_t],[a]]);

    #variance in the prediction
    P_k = np.dot(np.dot(A,P_k),A.T);

    Q = np.array([[0,0,0,0,0,0]
                ,[0,0,0,0,0,0]
                ,[0,0,2.5e-1, 0, 0, 0]
                ,[0,0,0,2.5e-1, 0, 0]
                ,[0,0,0,0, 2.5e-1, 0]
                ,[0,0,0,0, 0, 2.5e-1]])
    # Q = np.dot(np.dot(F, Q), F.T)
        
    P_k = P_k + Q

    R_k = np.array([[0.25e-2,0],[0,0.25e-2]])
    H = np.array([[1.0,0,0,0,0,0],[0,1,0,0,0,0]]);

    #Compute the Kalman Gain, using the prediction from the previous state and measurement from camera
    if detect==1:
        KG = np.dot(np.dot(P_k, H.T),np.linalg.inv(np.dot(np.dot(H,P_k),H.T) + R_k))
        X_new = X_new_pred + np.dot(KG,goal_pred - np.array([[X[0,0]],[X[1,0]]]))
        X = X_new
        P_k = np.dot(np.eye(KG.shape[0])- np.dot(KG, H), P_k)
        P_k = (P_k + P_k.T)/2
    if get_vel_flag:
        R_k = np.array([[0.025,0],[0,0.025]])
        H=np.array([[0,0,-v*sin(theta),0,cos(theta),0],[0,0,v*cos(theta),0,sin(theta),0]]);
        KG = np.dot(np.dot(P_k, H.T),np.linalg.inv(np.dot(np.dot(H,P_k),H.T) + R_k))
        X_new = X_new_pred + np.dot(KG,np.array([[v1],[v2]])-np.array([[v*cos(theta)],[v*sin(theta)]]))
        X = X_new
        P_k = np.dot(np.eye(KG.shape[0])- np.dot(KG, H), P_k)
        P_k = (P_k + P_k.T)/2

    if float(X[4,0])<0:
            X[4,0]=0;
    #Send the data
    print(msg)
    msg.pos.x = float(X[0,0])
    msg.pos.y = float(X[1,0])
    msg.pos.z = 0.393242
    msg.theta = float(X[2,0])
    msg.omegaz = float(X[3,0])
    msg.speed = float(X[4,0])
    msg.detected = detect
    pub.publish(msg)

    rospy.loginfo("states [xr,yr,vr,theta,om] : \n %s \n \n",np.array([xr,yr,np.sqrt(v_xr**2 + v_yr**2),yawr, v_yaw_r]))
    rospy.loginfo("states [xhat,yhat,vhat,thetahat,omhat] : \n %s \n \n",np.array([X[0,0],X[1,0],X[4,0],X[2,0],X[3,0]]))
    rospy.loginfo("states [xq,yq,vq,thetaq,omq,zq, pitchq,rollq] : \n %s \n \n",np.array([x,y,np.sqrt(v_x**2 + v_y**2),yaw, v_yaw,z,pitch,roll]))
    rospy.loginfo("states x,y measurements : \n %s \n \n",goal_pred)
    rospy.loginfo("states covariance : \n %s \n \n"%(np.array2string(np.round(P_k.flatten(),2))[2:-1]))

    f = open("rover_ekf.txt","a+");
    f.write("%s %s %s %s %s %s %s %s %s %s %s" %(xr,yr,np.sqrt(v_xr**2 + v_yr**2),yawr, v_yaw_r,X[0,0],X[1,0],X[4,0],X[2,0],X[3,0],simtime))
    for num in P_k.flatten():
        f.write(" %s"%(num))
    f.write("\n");
    f.close()
def ReceiveTar(data):
    global i, now_cam, now_cam_p, v_x, v_y, v_z, v_roll, v_pitch, v_yaw, x, y, z, roll, pitch, yaw, goal_pred, Rot_body_to_inertial, goal_pred_var, detect, v1, v2, x1_prev, x2_prev, xr,yr,zr,yawr

    get_vel_flag=False;
    
    Rot = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
        ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
        ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot = np.dot(np.array([[cos(np.pi/4), 0, -sin(np.pi/4)],[0, 1, 0],[sin(np.pi/4), 0, cos(np.pi/4)]]),Rot)
    R=Rot.T;
    vx = v_x
    vy = v_y
    xn = x
    yn = y
    zn = z
    xt_image = data.center.x
    yt_image = data.center.y
    radius = data.radius
    detect = data.detected
    now_cam = data.time

    if detect==0:
        rospy.loginfo(detect)
        pass
    else:
        #Get time
        del_t = now_cam-now_cam_p
        if del_t == 0:
            pass
        else:
            #Get the estimated positions from the camera measurement only
            zpush = np.dot(R,np.array([[0],[0],[-0.03]]))
            x1, x2 = get_position(xt_image, yt_image, xn, yn,z, R)
            x1 = x1+xn + zpush[0,0];
            x2 = x2+yn + zpush[1,0];

            #Average it to smoothen the data
            # x1 = 0.65*x1 + 0.35*x1_prev  
            # x2 = 0.65*x2 + 0.35*x2_prev   
            # x1_prev = x1
            # x2_prev = x2
 
            #Set these new values as the new predicted values from the camera measurement
            #Velocity is estimated by get_velociy fn which is called in const time intervals at the end
            goal_pred = np.array([[x1]
                                ,[x2]])

            #Set the variance in the measurements
            # goal_pred_var = np.array([[np.random.normal(0, 0.3*1.1**(float(img[0])*0.25/(z+0.0001))), 0, 0, 0]
            #                         ,[0, np.random.normal(0, 1+12*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y)+1/(0.25+abs(z-0.0))), 0, 0]
            #                         ,[0, 0, np.random.normal(0, 0.3*1.1**(float(img[1])*0.25/(z+0.0001))), 0]
            #                         ,[0, 0, 0, np.random.normal(0, 1+12*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y)+1/(0.25+abs(z-0.0)))]])

            now_cam_p = data.time
            i+=1
        
def get_position(xt, yt, xn, yn,zn,R):
    global vert_fov, hori_fov

    # Find directional vectors corresponding to (0,0), (w,0), (0,h), (w,h), (w/2,h/2) in the image plane using the fov of camera
    # Camera oriented at 45 deg
    # key_points_dir_body = np.array([[cos(np.pi/4-vert_fov)*cos(hori_fov), cos(np.pi/4-vert_fov)*cos(-hori_fov), cos(np.pi/4+vert_fov)*cos(hori_fov), cos(np.pi/4+vert_fov)*cos(-hori_fov), cos(np.pi/4)]
    #                                 ,[sin(hori_fov), sin(-hori_fov), sin(hori_fov), sin(-hori_fov), 0]
    #                                 ,[-sin(np.pi/4-vert_fov)*cos(hori_fov), -sin(np.pi/4-vert_fov)*cos(-hori_fov), -sin(np.pi/4+vert_fov)*cos(hori_fov), -sin(np.pi/4+vert_fov)*cos(-hori_fov), -sin(np.pi/4)]])
    
    # #Convert these vectors to the global frame
    # key_points_dir_global = np.dot(R, key_points_dir_body)

    # #Find the (x,y) coordinate where they intersect the plane of z=0.393242, which is the height of the rover
    # for i in range(len(key_points_dir_global[0])):
    #     t1 = float(key_points_dir_global[0][i])*(0.393242-z)/float(key_points_dir_global[2][i])
    #     t2 = float(key_points_dir_global[1][i])*(0.393242-z)/float(key_points_dir_global[2][i])

    #     key_points_dir_global[0][i] = t1*cos(yaw) - t2*sin(yaw) + xn
    #     key_points_dir_global[1][i] = t1*sin(yaw) + t2*cos(yaw) + yn
    #     key_points_dir_global[2][i] = 0.393242

    # #Compute the matrices M1 and M2 for computing the Homography Matrix M using SVD
    # #refer https://classroom.udacity.com/courses/ud955/lessons/3839218552/concepts/30084186810923
    # M1 = np.array([[float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0, 0, 0, 0]
    #             ,[0, 0, 0, float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0]
    #             ,[float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0, -1280*float(key_points_dir_global[0][1]), -1280*float(key_points_dir_global[1][1]), -1280*1]
    #             ,[0, 0, 0, float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0]
    #             ,[float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, 0, 0, 0, 0, 0, 0]
    #             ,[0, 0, 0, float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, -720*float(key_points_dir_global[0][2]), -720*float(key_points_dir_global[1][2]), -720*1]
    #             ,[float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, 0, 0, 0, -1280*float(key_points_dir_global[0][3]), -1280*float(key_points_dir_global[1][3]), -1280*1]
    #             ,[0, 0, 0, float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, -720*float(key_points_dir_global[0][3]), -720*float(key_points_dir_global[1][3]), -720*1]
    #             ,[float(key_points_dir_global[0][4]), float(key_points_dir_global[1][4]), 1, 0, 0, 0, -640*float(key_points_dir_global[0][4]), -640*float(key_points_dir_global[1][4]), -640*1]])

    # M2 = np.array([[xt]
    #             ,[yt]
    #             ,[1]])
        
    # U, D, V = np.linalg.svd(M1)
    # M = np.reshape(V[len(V)-1], (3,3))
    # M = np.linalg.inv(M)

    # #compute the positions
    # w1 = float(np.dot(M[0], M2)/np.dot(M[2], M2))
    # w2 = float(np.dot(M[1], M2)/np.dot(M[2], M2))
    
    x1 = 1280./2.-xt
    y1 = 720./2.-yt

    # print("these are the states : ", x1,y1)
    wvecbody = np.array([[1280.0/2.0/tan(hori_fov/2)],[x1],[y1]])
    wvec = np.dot(R,wvecbody)    
    # print("this is wvec_body", wvecbody)
    # print("this is wvec", wvec)
    wvec = wvec/wvec[2,0]*(0.393242-zn);
    # wvec = wvec + 

    w1 = wvec[0,0];
    w2 = wvec[1,0];

    return w1, w2

def get_velocity(event):
    global u1_prev, u2_prev, u3_prev, v1, v2, v1_prev, v2_prev, get_vel_flag

    #The function is called at const intervals
    dt = 0.25

    #Get the current positions
    w1 = float(goal_pred[0])
    w2 = float(goal_pred[1])

    #Calculate the velocities
    v1_n = (w1-u1_prev)/dt
    v2_n = (w2-u2_prev)/dt

    #Average the estimated velocity to smoothen it
    v1 = 0.6*v1_n+0.4*v1_prev    
    v2 = 0.6*v2_n+0.4*v2_prev

    #Update the previous velocity values
    v1_prev = v1
    v2_prev = v2    

    u1_prev = w1
    u2_prev = w2

    get_vel_flag = False;

    rospy.loginfo('[x,y,vx,vy]: %s \n\n', np.array([w1,w2,v1,v2]))

def time_callback( dat):
        global simtime
        # "Store" the message received.
        simtime = dat.clock.secs + dat.clock.nsecs*10**(-9);

def callback2(info):
    global xr, yr, zr, rollr, pitchr, yawr, Rot_body_to_inertial_r, Rot_inertial_to_body_r, v_roll_r, v_pitch_r, v_yaw_r, v_xr, v_yr, v_zr

    #Get the data from mavros
    #!!!!!!!!!!!        MAKE SURE TO CHECK THE AXES AS THEY ARE ALL INVERTED IN ALL OF MY CODES      !!!!!!!!!!!!!!!!
    xr = info.pose.pose.position.y + 20;
    yr = -info.pose.pose.position.x
    zr = 0.393242;

    va = info.twist.twist.linear.y
    vb = -info.twist.twist.linear.x
    vc = info.twist.twist.linear.z
    
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w
    q_new = tf.transformations.quaternion_multiply([0,0,-sin(np.pi/4),cos(np.pi/4)],[a1,b1,c1,d1])
    # q_new = tf.transformations.quaternion_multiply(q_new,[0,0,sin(np.pi/4),cos(np.pi/4)])
    a1 = q_new[0];
    b1 = q_new[1];
    c1 = q_new[2];
    d1 = q_new[3];
    # temp1 = a1;
    # a1=b1;
    # b1=-temp1;
    rollr, pitchr, yawr = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])
    # tempr = copy.deepcopy(rollr);
    # rollr = copy.deepcopy(pitchr);
    # pitchr = -copy.deepcopy(tempr);
    # yawr = yawr+np.pi/2;
    yawr = atan2(sin(yawr),cos(yawr));
    #yaw = yaw-np.pi/2
    #if yaw<np.pi/2:
    #    yaw = yaw+2*np.pi/2
    #if yaw>np.pi/2:
    #    yaw = yaw-2*np.pi/2

    Rot_body_to_inertial_r = np.array([[cos(yawr)*cos(pitchr), -sin(yawr)*cos(rollr)+sin(rollr)*sin(pitchr)*cos(yawr), sin(yawr)*sin(rollr)+cos(rollr)*cos(yawr)*sin(pitchr)]
                                    ,[sin(yaw)*cos(pitchr), cos(yawr)*cos(rollr)+sin(rollr)*sin(pitchr)*sin(yawr), -sin(rollr)*cos(yawr)+sin(yawr)*sin(pitchr)*cos(rollr)]
                                    ,[-sin(pitchr), cos(pitchr)*sin(rollr), cos(pitchr)*cos(rollr)]])
    Rot_inertial_to_body_r = Rot_body_to_inertial.transpose()
    
    v_roll_r = info.twist.twist.angular.y
    v_pitch_r = -info.twist.twist.angular.x
    v_yaw_r = info.twist.twist.angular.z

    v2 = np.array([[v_roll_r]
                ,[v_pitch_r]
                ,[v_yaw_r]])

    v1 = np.array([[va]
                ,[vb]
                ,[vc]])

    v2 = np.dot(Rot_body_to_inertial, v2)
    v1 = np.dot(Rot_body_to_inertial, v1)

    v_roll_r = float(v2[0])
    v_pitch_r = float(v2[1])
    v_yaw_r = float(v2[2])

    v_xr = float(v1[0])
    v_yr = float(v1[1])
    v_zr = float(v1[2])


def callback(info):
    global x, y, z, roll, pitch, yaw, Rot_body_to_inertial, Rot_inertial_to_body, v_roll, v_pitch, v_yaw, v_x, v_y, v_z

    #Get the data from mavros
    #!!!!!!!!!!!        MAKE SURE TO CHECK THE AXES AS THEY ARE ALL INVERTED IN ALL OF MY CODES      !!!!!!!!!!!!!!!!
    x = info.pose.pose.position.y
    y = -info.pose.pose.position.x
    z = info.pose.pose.position.z

    va = info.twist.twist.linear.y
    vb = -info.twist.twist.linear.x
    vc = info.twist.twist.linear.z
    
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w
    q_new = tf.transformations.quaternion_multiply([0,0,-sin(np.pi/4),cos(np.pi/4)],[a1,b1,c1,d1])
    # q_new = tf.transformations.quaternion_multiply(q_new,[0,0,-sin(np.pi/4),cos(np.pi/4)])
    a1 = q_new[0];
    b1 = q_new[1];
    c1 = q_new[2];
    d1 = q_new[3];

    # temp1 = a1;
    # a1=b1;
    # b1=-temp1;
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])
    # temp = copy.deepcopy(roll);
    # roll = copy.deepcopy(pitch);
    # pitch = -copy.deepcopy(temp);
    # yaw = yaw+np.pi/2;
    yaw = atan2(sin(yaw),cos(yaw));
    #yaw = yaw-np.pi/2
    #if yaw<np.pi/2:
    #    yaw = yaw+2*np.pi/2
    #if yaw>np.pi/2:
    #    yaw = yaw-2*np.pi/2

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()
    
    v_roll = info.twist.twist.angular.y
    v_pitch = -info.twist.twist.angular.x
    v_yaw = info.twist.twist.angular.z

    v2 = np.array([[v_roll]
                ,[v_pitch]
                ,[v_yaw]])

    v1 = np.array([[va]
                ,[vb]
                ,[vc]])

    v2 = np.dot(Rot_body_to_inertial, v2)
    v1 = np.dot(Rot_body_to_inertial, v1)

    v_roll = float(v2[0])
    v_pitch = float(v2[1])
    v_yaw = float(v2[2])

    v_x = float(v1[0])
    v_y = float(v1[1])
    v_z = float(v1[2])
                
def listener():
    rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.Subscriber("/quad/local_position/odom", Odometry, callback)
    rospy.Subscriber("/rover/local_position/odom", Odometry, callback2)
    rospy.Subscriber("/clock", Clock, time_callback)

    timer=rospy.Timer(rospy.Duration(10/1000.0),kalmanfunc)
    f = open("rover_ekf.txt","w");
    f.write("");
    f.close();
    if include_vel_meas:
        timer2=rospy.Timer(rospy.Duration(250/1000.0),get_velocity)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass