#! /usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from scipy.signal import argrelextrema
import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

SAMPLES = 60
START = 1
SENSOR_READINGS = np.array([0,0,0,0,0,0])
line_scans = []

plt.ion()



LIM = 10
XLIM = 10
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)
ax.set_thetamin(0)
ax.set_thetamax(180)
ax.set_rlim(0)
ax.set_rticks([2, 4, 6, 8, 10])  # Less radial ticks
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)
ax.legend()
line_scans, = ax.plot([], [], 'r.', alpha=0.9, ms=5 )
line_targets, = ax.plot([], [], 'k.', alpha=0.9, ms=5 )
def clbk_laser(msg):
    """ call back to process laser data"""
    global SENSOR_READINGS, line_scans,SAMPLES,START

    
    data = np.array(msg.ranges)
    reading_indeces = argrelextrema(np.array(msg.ranges), np.less)
    reading_indeces = np.array(reading_indeces).reshape(-1)
    bearings = (reading_indeces * np.pi / SAMPLES) 
    thetas = (np.arange(len(data)) * np.pi /SAMPLES)
    ranges = np.array(msg.ranges, dtype=np.float)[reading_indeces.astype(int)]
    line_scans.set_data(thetas, data)
    #line_targets.set_data(bearings, ranges)
    #ax.set_thetamin(0)
    #ax.set_thetamax(360)

    SENSOR_READINGS = np.hstack((bearings, ranges))
    #rospy.loginfo(SENSOR_READINGS)
    
    sensor_readings = np.array(SENSOR_READINGS).reshape(2, -1)
    b = sensor_readings[0]
    r = sensor_readings[1]
    line_targets.set_data(b, r)

def main():
    """ main method declares publishers and subscribers """
    global SENSOR_READINGS
    rospy.init_node('reading_laser')
    sub = rospy.Subscriber("/m2wr/laser/scan", LaserScan, clbk_laser)
    pub = rospy.Publisher('/range_readings', numpy_msg(Floats), queue_size=10)
    rate = rospy.Rate(5)

    print("Waiting for gazebo to start")
    time.sleep(5)
    print("Starting laser scan")
    while not rospy.is_shutdown():
        pub.publish(np.asarray(SENSOR_READINGS, dtype='float32'))
        rospy.loginfo_once('from sensor {}'.format(SENSOR_READINGS))
        fig.canvas.draw()
        fig.canvas.flush_events()
        rate.sleep()

if __name__ == '__main__':
    main()
