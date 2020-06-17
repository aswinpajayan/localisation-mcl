#!/usr/bin/env python
import time
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


T_MINUS_1 = 0
NUM = 40
START = 1
PREV_CMD = [0, 0, 0]
#PARTICLES = (np.random.rand(NUM * 3) * 6)- 3
PARTICLES = np.zeros(NUM * 3)


def cb_mcl(data):
    """ callback to perform montecarlo localisation
    triggered when controller issues a motion command
    : data: /cmd_vel - geometry_msgs.Twist """
    global  PARTICLES, NUM, T_MINUS_1, START, PREV_CMD
    now = rospy.get_time()
    delta_t = now - T_MINUS_1
    if(START == 1):
        delta_t = 0
        START = 0
    lin_vel, ang_vel = PREV_CMD[0], PREV_CMD[1]
    x = np.arange(NUM)
    y = np.arange(NUM, 2*NUM)
    phi = np.arange(2*NUM, 3*NUM)
    #motion update
    PARTICLES[x] = PARTICLES[x] + lin_vel * np.cos(PARTICLES[phi]) * delta_t
    PARTICLES[y] = PARTICLES[y] + lin_vel * np.sin(PARTICLES[phi]) * delta_t
    PARTICLES[phi] = PARTICLES[phi] - ang_vel * delta_t
    #Storing motion commands
    lin_vel = data.linear.x
    ang_vel = data.angular.z
    PREV_CMD = [lin_vel, ang_vel]
    T_MINUS_1 = now


def process():
    """ Routine Tasks """
    global PARTICLES
    rospy.init_node('Localiser_Node_MCL')
    pub = rospy.Publisher('/particles', numpy_msg(Floats), queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cb_mcl)
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
