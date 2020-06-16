#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from quat2euler import quat2euler


import numpy as np
import time
from math import sin, cos, atan2, pi

x = np.array([0,6,-3,0,0],dtype=np.float)
y = np.array([0,4,3,-1,0],dtype=np.float)
pose = [0.0, 0.0, 0.0]
### This parameter controls how close we want to generate the waypoints
K_samp = 0.07

### This function will give access to robots current position in `pose` variable
def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    

### this function will help in sampling the waypoints
def get_waypoint(t):
    global K_samp,x,y       ## defined at line 17
    A =3 
    B =3
    a =1
    b =2
    #x  = A*cos(a*t*K_samp)
    #y  = B*sin(b*t*K_samp)
    return [x[t],y[t]] 

### Plot the waypoints :   uses K_samp defined in line 17
def plot_waypoints():
    global K_samp
    
    Num_Pts = int(2*pi/K_samp)+1    # Number of points in 1 complete iteration

    W = [get_waypoint(i) for i in range(Num_Pts)] # Sampling points
        

### Map angle to within [-pi, pi]
def map_angle(theta):
    if(theta > pi):
        theta = -(2*pi-theta)
    elif(theta < -pi):
        theta = (2*pi+theta)
    return theta
    

### Saturate control to reasonable values
def sat(x, Th):
    if x<-Th:
        return -Th
    if x > Th:
        return Th
    return x


def control_loop():
    ## Routine tasks
    rospy.init_node('turtlebot_trajectory_tracker')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/odom', Odometry, callback)
    rate = rospy.Rate(10) 
    
    #### Uncomment the line to save the Waypont plot
    ## plot_waypoints()
    ################################################

    ## SAMPLE 1 WAYPONT
    i = 0
    wp= get_waypoint(i)
    dist_error = 2  # some random value
    
    while not rospy.is_shutdown():
    
        ## When close to waypoint, sample a new waypoint
        v = 1.0
        w = 0.0
        if pose[0] > 1:
            v = 0.1
            w = 0.8
            if dist_error < 0.3:
                i = i+1
                v= 0.05
                w = 2.0
                wp = get_waypoint(i)
        


        ### Compute errors
        x_err = wp[0]-pose[0]
        y_err = wp[1]-pose[1]
        theta_ref = atan2(y_err, x_err)
        
        dist_error = (x_err**2 + y_err**2)**0.5
        
        theta_err = map_angle(theta_ref - pose[2])
        w = 0.0 if theta_err < np.pi/5 else 2.0

        ### Debug string 
        #print("\n heading:{:0.5f},\tref:{:0.5f},\terror:{:0.5f}".format(pose[2], theta_ref, theta_err))
        
        ### Apply the proportional control
        K1=0.2 ## not aggressive
        K2=0.4 ## aggressive
        
        
        velocity_msg = Twist()
        #velocity_msg.linear.x = sat(K1*dist_error*cos(theta_err), 0.25)
        #velocity_msg.angular.z = sat(K2*theta_err, 2.0)
        velocity_msg.linear.x = v
        velocity_msg.angular.z = w
        
        
        pub.publish(velocity_msg)
        if pose[0] > 5 or pose[0]  < -7:
            break
        
        #print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        control_loop()
    except rospy.ROSInterruptException:
        pass
