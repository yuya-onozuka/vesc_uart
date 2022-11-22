#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import numpy as np

if __name__ == '__main__':
    rospy.init_node("sine_wave_publisher", anonymous=True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher("bldc_current", Float32, queue_size=10)
    signal = Float32()

    amplitude = 1
    frequency = 1

    while not rospy.is_shutdown():
        time = rospy.Time.now().to_sec()
        signal.data = amplitude*np.sin(2*np.pi*frequency*time)
        pub.publish(signal)
        rate.sleep()