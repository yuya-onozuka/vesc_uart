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
        self.current_capacity = 0
        self.scale_factor = 10000

        self.percentage = Int8()
        self.percentage.data = self.current_capacity / self.max_capacity

    def callback(self, data):
        current = data.data
        if current < 0:
            charged_capacity = self.scale_factor*current/self.freq_current_data/3600 #[Ah]
            self.current_capacity = self.current_capacity + charged_capacity
        

        self.percentage.data = int(abs(self.current_capacity / self.max_capacity))
        rospy.loginfo("percentage = %f", abs(self.current_capacity / self.max_capacity))
        self.pub.publish(self.percentage)



if __name__ == '__main__':
    rospy.init_node("calc_charged_capacity")
    rate = rospy.Rate(10)

    publisher = CalcChargedCapacity()

    while not rospy.is_shutdown():
        rate.sleep()
