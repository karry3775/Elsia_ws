#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry  # to get the laser cross track and abs yaw
from std_msgs.msg import String  # to get the block_val message
from geometry_msgs.msg import Twist  # for cmd_vel
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math as m
import time

rospy.init_node("coarse2fine_act_prop_node")

aisle_pub = rospy.Publisher("/aisle_odom", Odometry, queue_size=10)

# global variables
abs_yaw = 0.0
ct_dist = 0.0
f = 63.06  # 50 # mm
alpha = 0.264583  # 0.20977
lat = 4.0  # 7.6  # m (total cross track distance)
column_gap = 2.25  # m

# Global estimates
X_aisle = []
Y_aisle = []
odom_depth = 0.0
depth_thresh = column_gap / 3
last_laser_odom_x = 0.0
vel = 0.0
last_time_stamp = time.time()


def laser_cb(msg):
    global ct_dist
    y = msg.pose.pose.position.y
    ct_dist = -y


def aisle_ct_cb(msg):
    global ct_dist
    y = msg.pose.pose.position.y
    ct_dist = -y


def abs_yaw_cb(msg):
    global abs_yaw
    ori = msg.pose.pose.orientation
    (_, _, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    abs_yaw = yaw


def cmd_vel_cb(msg):
    global vel
    vel = msg.linear.x


def getDepth(l_r, px):
    camera_front_offset = 0.1
    ct_camera = ct_dist - (camera_front_offset) * m.sin(abs_yaw)
    if l_r == "l":
        print("it is l")
        # find xa and ya
        xa = px * (alpha/1000) * m.cos(abs_yaw) - \
            (f/1000) * m.sin(abs_yaw) + ct_camera
        ya = px * (alpha/1000) * m.sin(abs_yaw) + (f/1000) * m.cos(abs_yaw)
        depth = ((-(lat/2) - ct_camera) * ya) / (xa - ct_camera)
    else:
        # find xa and ya
        xa = px * (alpha/1000) * m.cos(abs_yaw) - \
            (f/1000) * m.sin(abs_yaw) + ct_camera
        ya = px * (alpha/1000) * m.sin(abs_yaw) + (f/1000) * m.cos(abs_yaw)
        print("it is r")
        depth = (((lat/2) - ct_camera) * ya) / (xa - ct_camera)

    depth = depth + (camera_front_offset) * m.cos(abs_yaw)
    print("depth value for abs_yaw = {}, ct_camera = {}, px = {} is: {}".format(
        abs_yaw, ct_camera, px, depth))
    return depth


def ignoreErratic(odom_depth, odom_depth_1, odom_depth_0):
    if abs(odom_depth - odom_depth_1) > 2:
        odom_depth = odom_depth_0
    else:
        odom_depth = odom_depth_1
    return odom_depth


def block_val_cb(msg):
    global X_aisle, Y_aisle, odom_depth, last_laser_odom_x, last_time_stamp

    ##################Find delta based on cmd_vel###############################
    cur_time_stamp = time.time()
    delta_time = cur_time_stamp - last_time_stamp
    last_time_stamp = cur_time_stamp
    delta_x = vel * delta_time * m.cos(abs_yaw)
    ############################################################################
    # msg is of the type string separated by commas --> block_count, px, l_r
    data = msg.data.split(",")
    print("####################################################")
    # print("data: {}".format(data))
    block_count = float(data[0])
    px = float(data[1])  # in pixels which is basically the x value
    l_r = data[2]
    px0 = float(data[3])  # second last value
    l_r0 = data[4]  # second last value

    print("original px was: {}".format(px))
    px = px - 200
    px0 = px0 - 200

    depth = getDepth(l_r, px)
    depth0 = getDepth(l_r0, px0)

    odom_depth_1 = (block_count * column_gap) - depth + column_gap
    odom_depth_0 = ((block_count - 1) * column_gap) - depth0 + column_gap

    odom_depth_final = ignoreErratic(odom_depth, odom_depth_1, odom_depth_0)

    print("prev_odom_depth : {}, odom_depth_0 : {}, odom_depth_1 : {}".format(
        odom_depth, odom_depth_0, odom_depth_1))
    # manually setting odom_depth to be

    weight = 0.98
    odom_depth = (1 - weight) * odom_depth_final + \
        weight * (odom_depth + delta_x)

    print("depth : {}, odom_depth : {}".format(depth, odom_depth))
    # append to the trajectory estimates
    Y_aisle.append(-ct_dist)
    X_aisle.append(odom_depth)

    # publish to aisle_odom
    q = quaternion_from_euler(0, 0, abs_yaw)

    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = odom_depth
    odom_msg.pose.pose.position.y = -ct_dist
    # putting raw aisle odometry as the z value
    odom_msg.pose.pose.position.z = 0.0#odom_depth_1 (we were obviously using this for some analytics, but for lets put the correct value here)
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id="/odom"
    odom_msg.child_frame_id="/aisle_link"

    aisle_pub.publish(odom_msg)


if __name__ == "__main__":
    try:
        abs_yaw_sub = rospy.Subscriber(
            "/abs_orientation_odom", Odometry, abs_yaw_cb)  # using the ceil's yaw
        # abs_yaw_sub = rospy.Subscriber("/ground_truth/state", Odometry, abs_yaw_cb) # using gt yaw
        # laser_sub = rospy.Subscriber("/odom_rf2o_corrected_ceil", Odometry, laser_cb)# using the ceil's y(ct_dist)
        # using the aisle cross track
        aisle_ct_sub = rospy.Subscriber("/aisle_ct", Odometry, aisle_ct_cb)
        # laser_sub = rospy.Subscriber("/ground_truth/state", Odometry, laser_cb) # using gt y(ct_dist)
        block_val_sub = rospy.Subscriber("/block_val", String, block_val_cb)
        cmd_vel_sub = rospy.Subscriber("/jacky/cmd_vel", Twist, cmd_vel_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
