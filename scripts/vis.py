#!/usr/bin/env python
from __future__ import division
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from quat2euler import quat2euler

from math import cos, sin, pi, hypot
import numpy as np

import matplotlib.pyplot as plt
#from controller import K_samp, get_waypoint

plt.ion()

K_samp = 0
def get_waypoint():
    return (0,0) 


LIM=8
XLIM=10
fig, ax = plt.subplots(1, 1)
#ax.set_autoscaley_on(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_xlim([-XLIM, XLIM])
ax.set_ylim([-LIM, LIM])
ax.grid()
ax.legend()
line_waypoints = [[3,0]]
line_poses = []
line_scans = []
cur_pose = []

map_true = np.array([[-5,4],[4,4],[2,-1],[2,-5],[-5,2],[-3,6]])
tracking_points = np.array([0,0])
line_waypoints, = ax.plot([], [], 'b.', label="waypoint", ms=5)
line_poses2, = ax.plot([],[],'r', lw=3 , alpha=0.9 )
line_poses, = ax.plot([],[],'ro', label="robot", ms=15.0, alpha=0.8)
line_scans, = ax.plot([],[],'k', alpha=0.9 )
ax.scatter(map_true[:,0],map_true[:,1], c='k',marker='*', label="landmarks")
track, = ax.plot([],[],'b:', lw=2, alpha=0.65)
    

def pose_listener( data):
    global line_poses, line_poses2, line_poses2, X_track, Y_track, tracking_points, line_waypoints, cur_pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    cur_pose = pose
    #print("Pose : {:0.2f},{:0.2f},{:0.2f}".format(pose[0], pose[1], pose[2]))
    delta = 0.50
    line_poses.set_data([pose[0]], [pose[1]])

    dx = delta*cos(pose[2])
    dy = delta*sin(pose[2])

    line_poses2.set_data([pose[0], pose[0]+dx], [pose[1], pose[1]+dy])
    
    ## save the tracks to plot later
    p = np.array([pose[0], pose[1]])
    if(np.linalg.norm(tracking_points[-1] - p) > 0.5):
        tracking_points  = np.vstack((tracking_points,[pose[0], pose[1]]))
       # print(tracking_points)
        line_waypoints.set_data(tracking_points[:,0], tracking_points[:,1])

def waypoint_listener( data):
    global line_waypoints
    waypoint = eval(data.data)
    #print("The type of data :{}",type(waypoint))
    line_waypoints.set_data( waypoint[0], waypoint[1])
            

def cb_scans( data):
    global line_scans,cur_pose
    bearings = data.data
    delta = 10
    for theta in bearings:
    	dx = delta*cos(theta + cur_pose[2])
    	dy = delta*sin(theta + cur_pose[2] )

    line_scans.set_data([cur_pose[0], cur_pose[0]+dx], [cur_pose[1], cur_pose[1]+dy])
    #print("The type of data :{}",type(waypoint))

def process():
    
    rospy.init_node('plotting_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, pose_listener)
    rospy.Subscriber("range_readings", numpy_msg(Floats), cb_scans)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        fig.canvas.draw()
        fig.canvas.flush_events()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Process started ")
        process()
    except rospy.ROSInterruptException:
        pass
    finally:
        #plt.plot(np.array(tracking_points)[:,0], np.array(tracking_points)[:,1], 'r:', alpha=0.7, label='trajectory')
        fig, ax = plt.subplots(1)
        Num_Pts = int(2*pi/K_samp)+1
        T = [i for i in range(Num_Pts)]
        #W = [get_waypoint(t) for t in T]
        #for w in W:
        #    ax.plot(w[0],w[1],'bs')
        #ax.plot(w[0], w[1], 'bs', label="Waypoints")

        #plt.xlabel('X (in meters)')
        #plt.ylabel('Y (in meters)')
        #plt.title('Robot Trajectory')
        #plt.savefig('Robot_Trajectory.png')
