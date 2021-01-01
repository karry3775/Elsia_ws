#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math as m

rospy.init_node("laser_odom_corrected_ceil_node_v002")

corrected_odom_pub = rospy.Publisher("/odom_rf2o_corrected_ceil", Odometry, queue_size = 10)
raw_odom_pub = rospy.Publisher("/odom_rf2o_uncorrected", Odometry, queue_size = 10)

x_raw = 0.0
y_raw = 0.0
yaw_raw = 0.0

x_c = 0.0
y_c = 0.0
yaw_c = 0.0

abs_yaw = 0.0


def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def abs_odom_cb(msg):
    global abs_yaw
    q = msg.pose.pose.orientation
    (_, _, abs_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

def laser_odom_cb(msg):
    global x_raw, y_raw, yaw_raw, x_c, y_c, yaw_c
    # extract pose
    pose = msg.pose.pose
    x = pose.position.x; y = pose.position.y; q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    line_angle = m.atan2((y - y_raw), (x - x_raw))
    beta = wrapToPi(line_angle - yaw_raw)
    d = m.sqrt((x - x_raw)**2  + (y - y_raw)**2)
    delta_x = d * m.cos(beta)
    delta_y = d * m.sin(beta)

    x_raw = x; y_raw = y; yaw_raw = yaw

    # convert the local deltas to global deltas
    delta_x_g = delta_x * m.cos(yaw_c) - delta_y * m.sin(yaw_c)
    delta_y_g = delta_x * m.sin(yaw_c) + delta_y * m.cos(yaw_c)

    yaw_c = abs_yaw
    x_c = x_c + delta_x_g
    y_c = y_c + delta_y_g

    ###
    q_raw = quaternion_from_euler(0,0,yaw_raw)
    odom_msg_raw = Odometry()
    odom_msg_raw.pose.pose.position.x = x_raw
    odom_msg_raw.pose.pose.position.y = y_raw
    odom_msg_raw.pose.pose.orientation.x = q_raw[0]
    odom_msg_raw.pose.pose.orientation.y = q_raw[1]
    odom_msg_raw.pose.pose.orientation.z = q_raw[2]
    odom_msg_raw.pose.pose.orientation.w = q_raw[3]

    odom_msg = Odometry()
    q_c = quaternion_from_euler(0,0,abs_yaw)
    odom_msg.pose.pose.position.x = x_c
    odom_msg.pose.pose.position.y = y_c
    odom_msg.pose.pose.orientation.x = q_c[0]
    odom_msg.pose.pose.orientation.y = q_c[1]
    odom_msg.pose.pose.orientation.z = q_c[2]
    odom_msg.pose.pose.orientation.w = q_c[3]

    t = rospy.Time.now()

    odom_msg.header.stamp = t
    odom_msg_raw.header.stamp = t

    corrected_odom_pub.publish(odom_msg)
    raw_odom_pub.publish(odom_msg_raw)


if __name__ == "__main__":
    try:
        rf2o_sub = rospy.Subscriber("/odom_rf2o", Odometry, laser_odom_cb)
        abs_odom_sub = rospy.Subscriber("/abs_orientation_odom", Odometry, abs_odom_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
