#!/usr/bin/env python
import time
import matplotlib.pyplot as plt
import numpy as np

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from sklearn.neighbors import NearestNeighbors
from quat2euler import quat2euler


# global constants
ANIMATE = True  # change to false to stop animating
NUM = 40
# MAP for the landmarks
MAP_L = np.array([[3, 1], [0, 5], [-2, 3], [-4, -1], [1, -2], [2, -1]])
# Parameters for measurment model
SIGMA_SQ = 0.01

# global variables and declarations
# pylint: disable-msg=C0325
# pylint: disable-msg=C0103

# to plot the targets and scans optionally
# based on GLOBAL_VARIABLE
if(ANIMATE):
    rospy.loginfo("interactive plotting is turned on")
    plt.ion()
    LIM = 6
    XLIM = 6
    fig, ax = plt.subplots(1, 1)
    # ax.set_autoscaley_on(True)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_xlim([-XLIM, XLIM])
    ax.set_ylim([-LIM, LIM])
    ax.grid()
    ax.legend()
    line_particles = []
    line_targets = []
    line_scans = []
    line_poses = []
    line_heading = []
    line_particles, = ax.plot([], [], 'g.', label="waypoint", ms=5)
    line_targets, = ax.plot([], [], 'r.', label="targets", ms=5)
    line_heading, = ax.plot([], [], 'r', lw=3, alpha=0.9, ms=15)
    line_poses, = ax.plot([], [], 'ro', label="robot", ms=15.0, alpha=0.8)
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
particles = np.zeros(NUM * 3)
# particles = (np.random.rand(NUM * 3) * 6)- 3
X = np.arange(NUM)
Y = np.arange(NUM, 2*NUM)
PHI = np.arange(2*NUM, 3*NUM)
#  pylint: enable-msg=C0103


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
    global line_poses, line_heading, line_particles  # pylint: disable-msg=C0103
    # pylint: enable-msg=W0603
    now = rospy.get_time()
    delta_t = now - t_minus_1
    if(START == 1):
        delta_t = 0
        lin_vel = data.linear.x
        ang_vel = data.angular.z
        prev_cmd = [lin_vel, ang_vel]
        t_minus_1 = now
        START = 0
        rospy.logdebug("MCL not started")
        return
    while(readings == []):
        pass
    lin_vel, ang_vel = prev_cmd[0], prev_cmd[1]
    noise = 0.001 * np.random.rand(3, NUM)
    # ------------motion prediction step-----------------------
    particles[X] = particles[X] + lin_vel * np.cos(particles[PHI]) * delta_t + noise[0]
    particles[Y] = particles[Y] + lin_vel * np.sin(particles[PHI]) * delta_t + noise[1]
    particles[PHI] = particles[PHI] - ang_vel * delta_t + noise[2]
    # Storing motion commands
    lin_vel = data.linear.x
    ang_vel = data.angular.z
    prev_cmd = [lin_vel, ang_vel]
    t_minus_1 = now
    # -----------measurement update : importance weight----------
    # 1. find endpoints of scan ,for each particle
    ranges, bearings, corr = readings
    weights = np.ones(NUM)
    for i in np.arange(len(ranges)):
        targets_x = particles[X] + ranges[i] * np.cos(-np.pi/2 + bearings[i] + particles[PHI])
        targets_y = particles[Y] + ranges[i] * np.sin(-np.pi/2 + bearings[i] + particles[PHI])
        targets = np.vstack((targets_x, targets_y)).reshape(-1, 2)
        # 2. find distance as metric to determine p(z|x,m)
        dist_sq = np.linalg.norm(targets - MAP_L[corr[i]]) ** 2
        # print(targets)
        weights = weights * np.exp(-0.5 * dist_sq / SIGMA_SQ)
    # 3. normalise the weights
    weights = weights / np.sum(weights)
    indeces = np.arange(NUM)
    # 4. importance sampling
    indeces = np.random.choice(indeces, NUM, p=weights)
    particles[X] = particles[indeces]
    particles[Y] = particles[NUM + indeces]
    particles[PHI] = particles[2*NUM + indeces]
    # routine_end_time = rospy.get_time()
    # routine_runtime = routine_end_time - now
    # rospy.logdebug('routine runtime :{}'.format(routine_runtime))

    rospy.loginfo_once("plotting feature is turned on")
    delta = 0.50
    line_poses.set_data([pose[0]], [pose[1]])
    delta_x = delta*np.cos(pose[2])
    delta_y = delta*np.sin(pose[2])
    line_heading.set_data([pose[0], pose[0]+delta_x], [pose[1], pose[1]+delta_y])
    line_particles.set_data(particles[X], particles[Y])


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
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        process()
    except rospy.ROSInterruptException:
        pass
