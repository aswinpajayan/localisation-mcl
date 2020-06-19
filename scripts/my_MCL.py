#!/usr/bin/env python
import time
import matplotlib.pyplot as plt
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry

from quat2euler import quat2euler
from sklearn.neighbors import NearestNeighbors


T_MINUS_1 = 0
NUM = 40
START = 1
PREV_CMD = [0, 0, 0]

#PARTICLES [X,Y,PHI]
PARTICLES = np.zeros(NUM * 3)
#PARTICLES = (np.random.rand(NUM * 3) * 6)- 3
X = np.arange(NUM)
Y = np.arange(NUM, 2*NUM)
PHI = np.arange(2*NUM, 3*NUM)

#MAP for the landmarks
MAP_L = np.array([[3, 1], [0, 5], [-2, 3], [-4, -1], [1, -2], [2, -1]])
READINGS = [np.zeros(3), np.zeros(3), np.zeros(3)] #for storing correspondeces

#pose is used just to provide data association
POSE = [0, 0, 0]

#Parameters for measurment model
SIGMA_SQ = 0.01

#visualisation
def cb_pose(data):
    """callback to store the pose
    note that the pose is used just to give data association
    : data : /odom nav_msgs.Odometry """
    global POSE
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    POSE = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x, y, z, w)[2]]


def cb_measurements(data):
    """ callback to collect sensor readings 
    :data : /range_readings numpy_msg(Floats)"""
    global READINGS
    readings = data.data
    readings = np.array(readings, dtype='float32').reshape(2, -1)
    ranges = readings[0]
    bearings = readings[1]
    targets_x = POSE[0] + ranges * np.cos(-np.pi/2 + bearings + POSE[2])
    targets_y = POSE[1] + ranges * np.sin(-np.pi/2 + bearings + POSE[2])
    targets = np.vstack((targets_x, targets_y)).reshape(-1, 2)
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(MAP_L)
    _, indeces = nbrs.kneighbors(targets)
    READINGS = [ranges, bearings, indeces]

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
    noise = 0.001 * np.random.rand(3, NUM)
    #------------motion prediction step-----------------------
    #

    PARTICLES[X] = PARTICLES[X] + lin_vel * np.cos(PARTICLES[PHI]) * delta_t + noise[0]
    PARTICLES[Y] = PARTICLES[Y] + lin_vel * np.sin(PARTICLES[PHI]) * delta_t + noise[1]
    PARTICLES[PHI] = PARTICLES[PHI] - ang_vel * delta_t +  noise[2]
    #Storing motion commands
    lin_vel = data.linear.x
    ang_vel = data.angular.z
    PREV_CMD = [lin_vel, ang_vel]
    T_MINUS_1 = now
    #-----------measurement update : importance weight----------
    # 1. find endpoints of scan ,for each particle
    ranges, bearings, corr = READINGS
    weights = np.ones(NUM)
    for i in np.arange(len(ranges)):
        targets_x = PARTICLES[X] + ranges[i] * np.cos(-np.pi/2 + bearings[i] + PARTICLES[PHI])
        targets_y = PARTICLES[Y] + ranges[i] * np.sin(-np.pi/2 + bearings[i] + PARTICLES[PHI])
        targets = np.vstack((targets_x, targets_y)).reshape(-1, 2)
        # 2. find distance as metric to determine p(z|x,m)
        dist_sq = np.linalg.norm(targets - MAP_L[corr[i]]) ** 2
        print(targets)
        weights = weights * np.exp(-0.5 * dist_sq / SIGMA_SQ)
    #3. normalise the weights
    weights = weights / np.sum(weights)
    indeces = np.arange(NUM)
    # 4. importance sampling
    indeces = np.random.choice(indeces, NUM, p=weights)
    PARTICLES[X] = PARTICLES[indeces]
    PARTICLES[Y] = PARTICLES[NUM + indeces]
    PARTICLES[PHI] = PARTICLES[2*NUM + indeces]


def process():
    """ Routine Tasks """
    global PARTICLES
    rospy.init_node('Localiser_Node_MCL')
    pub = rospy.Publisher('/particles', numpy_msg(Floats), queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cb_mcl)
    rospy.Subscriber('/odom', Odometry, cb_pose)
    rospy.Subscriber('/range_readings', numpy_msg(Floats), cb_measurements)
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
