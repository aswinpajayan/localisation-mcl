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

NUM_OF_PARTICLES = 40
PARTICLES = (np.random.rand(NUM_OF_PARTICLES * 3) * 6)- 3
#PARTICLES = np.zeros(NUM_OF_PARTICLES * 3)
pose = [0.0, 0.0, 0.0]

def cb_pose(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]

 
def cb_mcl(data):
    global pose, PARTICLES, NUM_OF_PARTICLES
    vx  = data.linear.x;
    vy  = data.linear.y;
    w = data.angular.z;
    v = hypot(vx, vy)
    r = v / w
    x_n = np.zeros(NUM_OF_PARTICLES)
    y_n = np.zeros(NUM_OF_PARTICLES)
    phi_n = np.zeros(NUM_OF_PARTICLES)
    p = PARTICLES.reshape(3, -1)
    x = p[0]
    y = p[1]
    phi = p[2]
    for i in np.arange(NUM_OF_PARTICLES):
        x_n[i] = x[i] - r * sin(phi[i]) + r * sin(phi[i] + w* 0.1)
        y_n[i] = y[i] + r * cos(phi[i]) - r * cos(phi[i] + w* 0.1)
        phi_n[i] = phi[i] + w* 0.1
    PARTICLES = np.hstack((x_n,y_n,phi_n))

   

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
    rospy.Subscriber('/odom', Odometry, cb_pose)
    rospy.Subscriber('/cmd_vel', Twist , cb_mcl)
    rate = rospy.Rate(2) 
    
    
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
