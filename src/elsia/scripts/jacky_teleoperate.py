#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import keyboard
import random

LIN_SPEED = 1.0
ANG_SPEED = 1.5
CUR_SPEED = 0.0
ACCEL = 0.0001

def cmd_pub():
    rospy.init_node("jacky_teleoperate_node")
    pub = rospy.Publisher("/jacky/cmd_vel", Twist, queue_size = 10)
    rate = rospy.Rate(50) # 50 Hz
    cmd = Twist()
    while not rospy.is_shutdown():
        print("listening for keys")
        # need to listen to arrow keys
        if keyboard.is_pressed("up"):
            cmd.linear.x = LIN_SPEED
            cmd.angular.z = 0.0
        elif keyboard.is_pressed("left"):
            cmd.linear.x = 0.0
            cmd.angular.z = ANG_SPEED
        elif keyboard.is_pressed("right"):
            cmd.linear.x = 0.0
            cmd.angular.z = -ANG_SPEED
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        pub.publish(cmd)
        rate.sleep()

def cmd_pub_acc():
    global CUR_SPEED
    rospy.init_node("jacky_acc_teleoperate_node")
    pub = rospy.Publisher("/jacky/cmd_vel", Twist, queue_size = 10)
    rate = rospy.Rate(50) # 50 Hz
    cmd = Twist()
    while not rospy.is_shutdown():
        print("CUR_SPEED : {}".format(CUR_SPEED))
        # need to listen to arrow keys
        if keyboard.is_pressed("up"):
            if CUR_SPEED < LIN_SPEED and CUR_SPEED + ACCEL <= LIN_SPEED:
                CUR_SPEED += ACCEL
            cmd.linear.x = CUR_SPEED
            cmd.angular.z = 0.0
        elif keyboard.is_pressed("left"):
            cmd.linear.x = CUR_SPEED
            cmd.angular.z = ANG_SPEED
        elif keyboard.is_pressed("right"):
            cmd.linear.x = CUR_SPEED
            cmd.angular.z = -ANG_SPEED
        else:
            if CUR_SPEED > 0 and CUR_SPEED - ACCEL >= 0:
                CUR_SPEED -= ACCEL
            cmd.linear.x = CUR_SPEED
            cmd.angular.z = 0.0

        pub.publish(cmd)
        # rate.sleep()

if __name__ == "__main__":
    try:
        cmd_pub_acc()
    except rospy.ROSInterruptException:
        pass
