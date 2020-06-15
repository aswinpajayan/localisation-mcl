#! /usr/bin/env python

import rospy
import roslib
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from scipy.signal import argrelextrema
import numpy as np

bearings = []

def clbk_laser(msg):
    global bearings
    # 720/5 = 144
   # regions = [ 
   #   min(msg.ranges[0:143]),
   #   min(msg.ranges[144:287]),
   #   min(msg.ranges[288:431]),
   #   min(msg.ranges[432:575]),
   #   min(msg.ranges[576:719]),
   #  ]
   # rospy.loginfo(regions)
    reading_indeces = argrelextrema(np.array(msg.ranges), np.less)
    bearings = (np.asarray(reading_indeces, dtype=np.float) * np.pi / 720) - np.pi/2
    rospy.loginfo(bearings)
    rospy.loginfo(reading_indeces)

def main():
    global bearings
    rospy.init_node('reading_laser')
    sub= rospy.Subscriber("/m2wr/laser/scan", LaserScan, clbk_laser)
    pub = rospy.Publisher('/range_readings', numpy_msg(Floats), queue_size=10)
    rate = rospy.Rate(5) 

    while not rospy.is_shutdown():
        pub.publish(np.array(bearings,dtype=np.float)) 
        rate.sleep()

if __name__ == '__main__':
    main()
