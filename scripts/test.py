#!/usr/bin/env python

import random
import rospy
from msl_skycrane_description.msg import CmdMotorPowerStamped


def p0(per, cont):
    if cont > 9:
        return random.getrandbits(9)%100
    else:
        return per

def p1(per, cont):
    if cont > 9:
        return random.getrandbits(9)%100
    else:
        return per

def p2(per, cont):
    if cont > 9:
        return random.getrandbits(9)%100
    else:
        return per

def p3(per, cont):
    if cont > 9:
        return random.getrandbits(9)%100
    else:
        return per

def talker():
    pub = rospy.Publisher('/skycrane/CmdMotorPower', CmdMotorPowerStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    now = rospy.Time.now()
    rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    rate.sleep()
    power = CmdMotorPowerStamped()
    i = 00
    j = 00
    k = 00
    l = 00

    cont = 0

    random.seed(None)
    # i = random.getrandbits(9)%100
    # j = random.getrandbits(9)%100
    # k = random.getrandbits(9)%100
    # l = random.getrandbits(9)%100

    # print i
    # print j
    # print k
    # print l

    # while not rospy.is_shutdown():
    #     now = rospy.Time.now()
    #     power.header.stamp.secs = now.secs
    #     power.header.stamp.nsecs = now.nsecs
    #     power.power.landing_engine00 = 75
    #     power.power.landing_engine01 = 75
    #     power.power.landing_engine10 = 75
    #     power.power.landing_engine11 = 75
    #     power.power.landing_engine20 = 75
    #     power.power.landing_engine21 = 75
    #     power.power.landing_engine30 = 75
    #     power.power.landing_engine31 = 75
    #     rospy.loginfo(power)
    #     pub.publish(power)
    #     rate.sleep()



    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        now = rospy.Time.now()
        power.header.seq = i
        power.header.stamp.secs = now.secs
        power.header.stamp.nsecs = now.nsecs
        power.power.landing_engine00 = p0(power.power.landing_engine00, cont)
        power.power.landing_engine01 = p0(power.power.landing_engine00, cont)
        power.power.landing_engine10 = p1(power.power.landing_engine11, cont)
        power.power.landing_engine11 = p1(power.power.landing_engine11, cont)
        power.power.landing_engine20 = p2(power.power.landing_engine20, cont)
        power.power.landing_engine21 = p2(power.power.landing_engine20, cont)
        power.power.landing_engine30 = p3(power.power.landing_engine30, cont)
        power.power.landing_engine31 = p3(power.power.landing_engine30, cont)
        rospy.loginfo(power)
        pub.publish(power)
        i = i + 1
        cont = cont + 1
        print cont
        print i
        rate.sleep()
        if cont > 10:
            cont = 0
        # if i > 100:
            # break

    # i = 00
    # while not rospy.is_shutdown():
    #     # hello_str = "hello world %s" % rospy.get_time()
    #     power.landing_engine00 = 00
    #     power.landing_engine01 = 00
    #     power.landing_engine10 = i
    #     power.landing_engine11 = i
    #     power.landing_engine20 = 00
    #     power.landing_engine21 = 00
    #     power.landing_engine30 = 00
    #     power.landing_engine31 = 00
    #     rospy.loginfo(power)
    #     pub.publish(power)
    #     i = i + 5
    #     rate.sleep()
    #     if i > 100:
    #         break

    # i = 00
    # while not rospy.is_shutdown():
    #     # hello_str = "hello world %s" % rospy.get_time()
    #     power.landing_engine00 = 00
    #     power.landing_engine01 = 00
    #     power.landing_engine10 = 00
    #     power.landing_engine11 = 00
    #     power.landing_engine20 = i
    #     power.landing_engine21 = i
    #     power.landing_engine30 = 00
    #     power.landing_engine31 = 00
    #     rospy.loginfo(power)
    #     pub.publish(power)
    #     i = i + 5
    #     rate.sleep()
    #     if i > 100:
    #         break
    #     # rospy.loginfo(hello_str)
    #     # pub.publish(hello_str)

    # i = 00
    # while not rospy.is_shutdown():
    #     # hello_str = "hello world %s" % rospy.get_time()
    #     power.landing_engine00 = 00
    #     power.landing_engine01 = 00
    #     power.landing_engine10 = 00
    #     power.landing_engine11 = 00
    #     power.landing_engine20 = 00
    #     power.landing_engine21 = 00
    #     power.landing_engine30 = i
    #     power.landing_engine31 = i
    #     rospy.loginfo(power)
    #     pub.publish(power)
    #     i = i + 5
    #     rate.sleep()
    #     if i > 100:
    #         break


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
