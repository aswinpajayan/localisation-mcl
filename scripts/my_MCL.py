#!/usr/bin/env python
import time
import cv2
import matplotlib.pyplot as plt
import numpy as np

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from sklearn.neighbors import NearestNeighbors
from quat2euler import quat2euler
from multi_gaussian import get_gaussian
from robot_models import get_likelihood_field
from lv_resample import re_sampling


# global constants
ANIMATE = True  # change to false to stop animating
NUM = 40
# MAP for the landmarks
MAP_L = np.array([[3, 1], [0, 5], [-2, 3], [-4, -1], [1, -2], [2, -1]])
# Parameters for measurment model
SIGMA_SQ = 0.25

YLIM = 6
XLIM = 6
# global variables and declarations
# pylint: disable-msg=C0325
# pylint: disable-msg=C0103

Z_XM = get_likelihood_field(12, MAP_L, "gaussian", [0.212, 0.1, 20])
a =np.sum(Z_XM)
while(np.isnan(a)):
    print("gaya")
    pass
# to plot the targets and scans optionally
# based on GLOBAL_VARIABLE
if(ANIMATE):
    rospy.loginfo("interactive plotting is turned on")
    plt.ion()
    fig, ax = plt.subplots(1, 1)
    # ax.set_autoscaley_on(True)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_xlim([-XLIM, XLIM])
    ax.set_ylim([-YLIM, YLIM])
    ax.grid()
    ax.legend()
    line_particles = []
    line_targets = []
    line_scans = []
    line_poses = []
    line_heading = []
    line_heading_p = []
    line_particles, = ax.plot([], [], 'g.', label="waypoint", ms=5)
    line_targets, = ax.plot([], [], 'r.', label="targets", ms=5)
    line_heading, = ax.plot([], [], 'r', lw=3, alpha=0.9, ms=15)
    line_heading_p, = ax.plot([], [], 'g', alpha=0.9, ms=15)
    line_poses, = ax.plot([], [], 'ro', label="robot", ms=15.0, alpha=0.8)
    ax.scatter(MAP_L[:, 0], MAP_L[:, 1], c='k', marker='*', label="landmarks")
    line_scans, = ax.plot([], [], 'k:')
else:
    rospy.loginfo("interactive plotting is turned off")
    rospy.loginfo("To turn on animation set ANIMATE=True in my_mcl.py")


t_minus_1 = 0
START = 1
prev_cmd = [0, 0, 0]

# pose is used just to provide data association
pose = [0, 0, 0]
# for storing correspondeces
readings = [np.zeros(3), np.zeros(3), np.zeros(3)]
# particles [X,Y,PHI]
#particles = np.zeros(NUM * 3)
particles = (np.random.rand(NUM * 3) * 2) - 1
X = np.arange(NUM)
Y = np.arange(NUM, 2*NUM)
PHI = np.arange(2*NUM, 3*NUM)
#  pylint: enable-msg=C0103


def measurement_model():
    """maximum likelihood field measurement model

    :MAP_L: land mark locations
    :returns: sets a global dictionary of discretised grid
    and thier likeihood values
    """
    num_of_gridpoints = 100
    scale = XLIM * 2 / num_of_gridpoints
    # kernel size
    k_size = np.round(np.sqrt(SIGMA_SQ) * 5 * scale)
    k_size = 40
    ml_field = np.zeros((XLIM, YLIM), dtype=np.float)
    x_axis = np.arange(-k_size, k_size)
    y_axis = np.linspace(-k_size, k_size)
    x_values, y_values = np.meshgrid(x_axis, y_axis)
    grid = np.empty(x_values.shape + (2,))
    grid[:, :, 0] = x_values
    grid[:, :, 1] = y_values
    mean = np.array([0, 0])
    cov = np.diag([10, 10])
    rospy.loginfo('covariance {}'.format(cov))
    kernel = get_gaussian(grid, mean, cov)
    kernel = kernel / np.max(kernel)
    # print(kernel)
    # cv2.imshow('kernel', kernel)


def map_angle(theta):
    """function to map angle between -pi to pi)"""
    if(theta > np.pi):
        theta = -(2*np.pi-theta)
    elif(theta < -np.pi):
        theta = (2*np.pi+theta)
    return theta


def cb_pose(data):
    """callback to store the pose
    note that the pose is used just to give data association
    : data : /odom nav_msgs.Odometry """
    # pylint: disable-msg=W0603
    global pose  # pylint: disable-msg=C0103
    # pylint: enable-msg=W0603
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,
            quat2euler(data.pose.pose.orientation.x,
                       data.pose.pose.orientation.y,
                       data.pose.pose.orientation.z,
                       data.pose.pose.orientation.w)[2]]


def cb_measurements(data):
    """ callback to collect sensor readings
    :data : /range_readings numpy_msg(Floats)"""
    # pylint: disable-msg=W0603
    global readings  # pylint: disable-msg=C0103
    # pylint: enable-msg=W0603
    readings = data.data
    readings = np.array(readings, dtype='float32').reshape(2, -1)
    ranges = readings[0]
    bearings = readings[1]
    targets_x = pose[0] + ranges * np.cos(-np.pi/2 + bearings + pose[2])
    targets_y = pose[1] + ranges * np.sin(-np.pi/2 + bearings + pose[2])
    targets = np.vstack((targets_x, targets_y)).reshape(-1, 2)
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(MAP_L)
    _, indeces = nbrs.kneighbors(targets)
    readings = [ranges, bearings, indeces]


def cb_mcl(data):
    """ callback to perform montecarlo localisation
    triggered when controller issues a motion command
    : data: /cmd_vel - geometry_msgs.Twist """
    #  pylint: disable-msg=W0603
    global particles, t_minus_1, START, prev_cmd, pose  # pylint: disable-msg=C0103
    global line_poses, line_heading, line_particles, line_scans  # pylint: disable-msg=C0103
    global line_heading_p  # pylint: disable-msg=C0103
    # pylint: enable-msg=W0603
    now = rospy.get_time()
    delta_t = now - t_minus_1
    if(START == 1):
        delta_t = 0
        t_minus_1 = now
        START = 0
        rospy.logdebug("MCL not started")
        return
    while(readings == []):
        pass
    lin_vel, ang_vel = data.linear.x, -data.angular.z
    noise = 0.2 * np.random.rand(3, NUM)
    # ------------motion prediction step-----------------------
    if(np.abs(ang_vel) > 0.01):
        r = lin_vel / ang_vel
        particles[X] = particles[X] - r * np.sin(particles[PHI]) + r * np.sin(particles[PHI] + ang_vel * delta_t )
        particles[Y] = particles[Y] + r * np.cos(particles[PHI]) - r * np.cos(particles[PHI] + ang_vel * delta_t )
    else:
        particles[X] = particles[X] + lin_vel * np.cos(particles[PHI]) * delta_t + noise[0]
        particles[Y] = particles[Y] + lin_vel * np.sin(particles[PHI]) * delta_t + noise[1]
    particles[PHI] = particles[PHI] + ang_vel * delta_t + 0.1 * noise[2]
    t_minus_1 = now
    # -----------measurement update : importance weight----------
    # 1. find endpoints of scan ,for each particle
    true_endpoints = np.array([0, 0])
    all_endpoints = np.array([0, 0])
    ranges, bearings, corr = readings
    weights = np.ones((NUM + 1), dtype=np.float)
    for i in np.arange(len(ranges)):
        endpoints = np.array([0, 0])
        # targets_x = particles[X] + ranges[i] * np.cos(-np.pi/2 + bearings[i] + np.array((map(map_angle, particles[PHI]))))
        # targets_y = particles[Y] + ranges[i] * np.sin(-np.pi/2 + bearings[i] + np.array((map(map_angle, particles[PHI]))))
        for j in np.arange(NUM):
            targets_x = particles[j] + ranges[i] * np.cos(-np.pi/2 + bearings[i] + particles[2*NUM+j])
            targets_y = particles[NUM+j] + ranges[i] * np.sin(-np.pi/2 + bearings[i] + particles[2*NUM+j])
            targets = np.vstack((targets_x, targets_y)).reshape(-1, 2)
            endpoints = np.vstack((endpoints, targets))
        true_targets_x = pose[0] + ranges[i] * np.cos(-np.pi/2 + bearings[i] + pose[2])
        true_targets_y = pose[1] + ranges[i] * np.sin(-np.pi/2 + bearings[i] + pose[2])
        true_targets = np.vstack((true_targets_x, true_targets_y)).reshape(-1, 2)
        # 2. find distance as metric to determine p(z|x,m)
        #dist_sq = np.array( [np.linalg.norm(endpoint - MAP_L[corr[i]]) ** 2 for endpoint in endpoints])
        # print(targets)
        # weights = weights * np.array([np.exp(-0.5 * x / SIGMA_SQ) if x > 0.25 else 1 for x in dist_sq])
        
        points = np.ceil(endpoints * 20).astype(int)
        prob = np.array([Z_XM[p[1] + 120, 120 - p[0]] for p in points], dtype=np.float)
        #p_sum = np.sum(prob)
        #if(np.isnan(p_sum)):
        #    rospy.logdebug("Nan detected in prob")
        #else:
        #    rospy.logdebug("---------prob----------{}---".format(p_sum))

        #weights = weights * prob
        #p_sum = np.sum(weights)
        #if(np.isnan(p_sum)):
        #    rospy.logdebug("Nan detected in weights")
        #else:
        #    rospy.logdebug("---------weights--------{}-----".format(p_sum))
        #    rospy.logdebug(weights)
        true_endpoints = np.vstack((true_endpoints, (pose[0], pose[1])))
        true_endpoints = np.vstack((true_endpoints, true_targets))
        all_endpoints = np.vstack((all_endpoints, endpoints))
    # 3. normalise the weights
    weights = weights[1:] / (np.sum(weights[1:]))
    indeces = np.arange(NUM)
    # 4. importance sampling
    indeces = np.random.choice(indeces, NUM, p=weights)
    #indeces = re_sampling(weights)
    rospy.logdebug_throttle(5, 'indeces after resampling {}'.format(indeces))
    particles[X] = particles[indeces]
    particles[Y] = particles[NUM + indeces]
    particles[PHI] = particles[2*NUM + indeces]

    if(ANIMATE):
        rospy.loginfo_once("plotting feature is turned on")
        delta = 0.50
        line_poses.set_data([pose[0]], [pose[1]])
        delta_x = delta*np.cos(pose[2])
        delta_y = delta*np.sin(pose[2])
        line_heading.set_data([pose[0], pose[0]+delta_x], [pose[1], pose[1]+delta_y])
        delta_x = delta*np.cos(particles[PHI])
        delta_y = delta*np.sin(particles[PHI])
        line_heading_p.set_data([particles[X], particles[X]+delta_x], [particles[Y], particles[Y]+delta_y])
        line_particles.set_data(particles[X], particles[Y])
        line_targets.set_data(all_endpoints[:, 0], all_endpoints[:, 1])
        line_scans.set_data(true_endpoints[:, 0], true_endpoints[:, 1])
        rospy.logdebug_throttle(5, true_targets)
    else:
        rospy.loginfo_once("interactive plotting is turned off")
        rospy.loginfo_once("To turn on animation set ANIMATE=True in my_mcl.py")


def process():
    """ Routine Tasks """
    # pylint: disable-msg=W0603
    global particles  # pylint: disable-msg=C0103
    # pylint: enable-msg=W0603
    rospy.init_node('Localiser_Node_MCL')
    pub = rospy.Publisher('/particles', numpy_msg(Floats), queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cb_mcl)
    rospy.Subscriber('/odom', Odometry, cb_pose)
    rospy.Subscriber('/range_readings', numpy_msg(Floats), cb_measurements)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(np.asarray(particles, dtype='float32'))
        fig.canvas.draw()
        fig.canvas.flush_events()
        rate.sleep()


if __name__ == '__main__':
    try:
        measurement_model()
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        process()
    except rospy.ROSInterruptException:
        pass
