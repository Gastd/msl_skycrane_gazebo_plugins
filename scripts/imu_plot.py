#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt
# import numpy as np


data = []


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear_acceleration.z)
    data.append(data.linear_acceleration.z)


def imu_plot():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Imu, callback)
    if len(data) > 5:
        plt.xlabel('time (s)')
        plt.ylabel('imu_z (m/s^2)')
        plt.title('About as simple as it gets, folks')
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    try:
        imu_plot()
    except rospy.ROSInterruptException:
        pass
