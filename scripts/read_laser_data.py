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
SENSOR_READINGS = np.array([0, 0, 0, 0, 0, 0])
THETAS = np.arange(SAMPLES) * np.pi / SAMPLES
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
ax.setxticks(np.pi/180. * np.linspace(0, 180, 8, endpoint=False))
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)
ax.legend()
line_scans, = ax.plot([], [], 'r.', alpha=0.9, ms=5 )
line_targets, = ax.plot([], [], 'k.', alpha=0.9, ms=5 )

def process_reading(msg):
    """Takes in raw readings and outputs point target location

    :msg : Sensor message from hokuyo controller sensor_msgs.msg.LaserScan
    :sensor_reading: sensor reading as triplet [range bearing correspondence] list
    """
    global SAMPLES
    data = np.array(msg.ranges)
    reading_indeces = argrelextrema(np.array(msg.ranges), np.less)
    reading_indeces = np.array(reading_indeces).reshape(-1)
    bearings = (reading_indeces * np.pi / SAMPLES) 
    ranges = np.array(msg.ranges, dtype=np.float)[reading_indeces.astype(int)]
    correspondence = [1, 2, 3]
    sensor_reading = [ranges, bearings, correspondence]
    return sensor_reading

def clbk_laser(msg):
    """ call back to process laser data"""
    global SENSOR_READINGS, line_scans, THETAS

    data = np.array(msg.ranges)
    ranges, bearings, correspondence = process_reading(msg)
    line_scans.set_data(THETAS, data)
    line_targets.set_data(bearings, ranges)

    SENSOR_READINGS = np.hstack((ranges, bearings, correspondence))
    #rospy.loginfo(SENSOR_READINGS)

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
