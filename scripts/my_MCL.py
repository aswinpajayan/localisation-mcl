#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

from quat2euler import quat2euler


import numpy as np
import time
from math import sin, cos, atan2, pi, hypot

T_MINUS_1 = 0
NUM = 40
START = 1
PREV_CMD = [0, 0, 0]
#PARTICLES = (np.random.rand(NUM * 3) * 6)- 3
PARTICLES = np.zeros(NUM * 3)
pose = [0.0, 0.0, 0.0]

def cb_pose(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]

 
def cb_mcl(data):
    global pose, PARTICLES, NUM, T_MINUS_1, START, PREV_CMD
    now = rospy.get_time()
    delta_t = now - T_MINUS_1
    if(START == 1):
        delta_t = 0
        START = 0
    lin_vel, vel_y, ang_vel = PREV_CMD[0], PREV_CMD[1], PREV_CMD[2]
    PARTICLES[:NUM] = PARTICLES[:NUM] + lin_vel * np.cos(PARTICLES[2*NUM:3*NUM]) * delta_t
    PARTICLES[NUM:2*NUM] = PARTICLES[NUM:2*NUM] + lin_vel * np.sin(PARTICLES[2*NUM:3*NUM]) * delta_t
    PARTICLES[2*NUM:3*NUM] = PARTICLES[2*NUM:3*NUM] - ang_vel * delta_t
    #motion update
    #for i in np.arange(NUM):
       # if(np.abs(ang_vel)  0.001):
       #     r = lin_vel / ang_vel
       #     x_n[i] = x[i] - (r * sin(phi[i])) + r * sin(phi[i] + ang_vel * delta_t) 
       #     y_n[i] = y[i] + (r * cos(phi[i])) - r * cos(phi[i] + ang_vel * delta_t)
       # else:
       #     rospy.loginfo(ang_vel)
        #x_n[i]= x[i] + lin_vel * np.cos(phi[i]) * delta_t
        #y_n[i] = y[i] + lin_vel * np.sin(phi[i]) * delta_t
        #phi_n[i] = map_angle(phi[i] - ang_vel * delta_t)
      #  theta = PARTICLES[2*NUM + i]
      #  PARTICLES[i] = PARTICLES[i] + lin_vel * np.cos(theta) * delta_t
      #  PARTICLES[NUM + i] = PARTICLES[NUM + i] + lin_vel * np.sin(theta) * delta_t
      #  PARTICLES[2*NUM + i] = map_angle(theta - ang_vel * delta_t)
        #x_n[i] = x[i] + 1 * np.cos(phi[i]) * delta_t
        #y_n[i] = y[i] + 1 * np.sin(phi[i]) * delta_t
        #phi_n[i] = phi[i] + 0.5 * delta_t
    #PARTICLES = np.hstack((x, y, phi))
    #Storing motion commands
    lin_vel = data.linear.x
    vel_y = data.linear.y
    ang_vel = data.angular.z
    PREV_CMD = [lin_vel, vel_y, ang_vel]
    T_MINUS_1 = now


def map_angle(theta):
    if(theta > pi):
        theta = -(2*pi-theta)
    elif(theta < -pi):
        theta = (2*pi+theta)
    return theta

def process():
    ## Routine tasks
    global PARTICLES
    rospy.init_node('Localiser_Node_MCL')
    pub = rospy.Publisher('/particles', numpy_msg(Floats), queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cb_mcl)
    rospy.Subscriber('/odom', Odometry, cb_pose)
    rate = rospy.Rate(1) 
    
    
    while not rospy.is_shutdown():
        pub.publish(np.asarray(PARTICLES, dtype='float32'))
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        process()
    except rospy.ROSInterruptException:
        pass
