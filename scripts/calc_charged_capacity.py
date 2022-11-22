#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int8, Float32

class CalcChargedCapacity:

    def __init__(self):
        rospy.Subscriber("bldc_current", Float32, self.callback)
        self.pub = rospy.Publisher("percentage", Int8, queue_size=10)

        self.max_capacity = 6 #[Ah]
        self.freq_current_data = 10 #[Hz]
        self.current_capacity = 3 # set initial percentage
        self.scale_factor = 10000

        self.percentage = Int8()

    def callback(self, data):
        current = data.data
        charged_capacity = self.scale_factor*current/self.freq_current_data/3600 #[Ah]
        self.current_capacity = self.current_capacity + charged_capacity
        
        percentage = self.current_capacity / self.max_capacity * 100
        if percentage <= 0:
            self.percentage.data = 0
        elif percentage >= 100:
            self.percentage.data = 100
        else:
            self.percentage.data = int(percentage)
        rospy.loginfo("percentage = %f", self.percentage.data)
        self.pub.publish(self.percentage)

if __name__ == '__main__':
    rospy.init_node("calc_charged_capacity")
    rate = rospy.Rate(10)

    publisher = CalcChargedCapacity()

    while not rospy.is_shutdown():
        rate.sleep()
