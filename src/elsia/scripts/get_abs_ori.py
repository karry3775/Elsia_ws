#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import math as m

# initialize the node
rospy.init_node("get_abs_ori_node")

# global variables
best_ori_estimate = 0.0
ini_angle_offset = 0.0

# create publishers
odom_pub = rospy.Publisher("/abs_orientation_odom", Odometry, queue_size = 10)
image_pub = rospy.Publisher("/considered_image", Image, queue_size = 10)


def wrap2Pi(theta):
    wrappedUpVal = m.atan2(m.sin(theta), m.cos(theta))
    return wrappedUpVal

def abs_ori_cb(msg):
    global best_ori_estimate
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        # crop out the excess image
        cv_image = cv_image[100:300, 100:300, :]
    except CvBridgeError as e:
        print("[INFO]: Error in obtaining image from CvBridge! Skipping frame!")
    else:
        # convert to gray
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # convert to edges
        edges = cv2.Canny(gray, 50, 150)
        cv2.imshow("edges", edges)
        cv2.waitKey(1)
        # convert to thresholded image
        ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        # extract hough lines
        lines = cv2.HoughLinesP(edges, 1, m.pi/180, 2, None, 20, 1)

        #list of [count, angle] pairs
        cnt_ang_pair = []

        # draw lines
        for i in range(lines.shape[0]):
            for line in lines[i]:
                pt1 = (line[0], line[1])
                pt2 = (line[2], line[3])
                cv2.line(cv_image, pt1, pt2, (255, 0, 0), 3)
                # calculate angle
                ang =  m.atan2(pt2[1]-pt1[1], pt2[0]-pt1[0])
                cnt_ang_pair.append([1, m.degrees(ang)])

        ###################### show the detected lines ########################
        cv2.imshow("frame", cv_image)
        cv2.waitKey(1)
        #######################################################################

        if len(cnt_ang_pair) != 0:
            # sort the cnt_ang_pair
            cnt_ang_pair.sort(key = lambda x:x[1])

            # bunch up the pairs based on predetermined threshold
            ang_thresh_deg = 1
            bunch = [cnt_ang_pair[0]]
            for i in range(1, len(cnt_ang_pair)):
                pairs = cnt_ang_pair[i]
                if abs(pairs[1] - bunch[-1][1]) < ang_thresh_deg:
                    # update the value and the count
                    new_count = bunch[-1][0] + 1
                    new_value = ((bunch[-1][1] * (new_count - 1) * 1.0) / new_count) + (pairs[1]*1.0) / new_count
                    bunch[-1] = [new_count, new_value]
                else:
                    # time to append
                    bunch.append(pairs)

            # sort bunch based on first value i.e. count
            bunch.sort(key = lambda x:x[0], reverse = True)

            print("The cnt_ang_pair list is: \n {} \n".format(cnt_ang_pair))
            print("The bunched up list is: \n {} \n".format(bunch))

            # use the first value of bunch
            f_ori = m.radians(bunch[0][1]) # in degrees
            f_ori1 = wrap2Pi(f_ori + m.radians(90) - ini_angle_offset)
            f_ori2 = wrap2Pi(f_ori + m.radians(-90) - ini_angle_offset)
            f_ori3 = wrap2Pi(f_ori + m.radians(180) - ini_angle_offset)

            # we need to find which has the smallest difference
            # f_ori, f_ori1 or f_ori2
            if(abs(wrap2Pi(best_ori_estimate - f_ori)) < abs(wrap2Pi(best_ori_estimate - f_ori1)) and abs(wrap2Pi(best_ori_estimate - f_ori)) < abs(wrap2Pi(best_ori_estimate - f_ori2)) and abs(wrap2Pi(best_ori_estimate - f_ori)) < abs(wrap2Pi(best_ori_estimate - f_ori3))):
                best_ori_estimate_temp = f_ori
            elif(abs(wrap2Pi(best_ori_estimate - f_ori1)) < abs(wrap2Pi(best_ori_estimate - f_ori)) and abs(wrap2Pi(best_ori_estimate - f_ori1)) < abs(wrap2Pi(best_ori_estimate - f_ori2)) and abs(wrap2Pi(best_ori_estimate - f_ori1)) < abs(wrap2Pi(best_ori_estimate - f_ori3))):
                best_ori_estimate_temp = f_ori1
            elif(abs(wrap2Pi(best_ori_estimate - f_ori2)) < abs(wrap2Pi(best_ori_estimate - f_ori)) and abs(wrap2Pi(best_ori_estimate - f_ori2)) < abs(wrap2Pi(best_ori_estimate - f_ori1)) and abs(wrap2Pi(best_ori_estimate - f_ori2)) < abs(wrap2Pi(best_ori_estimate - f_ori3))):
                best_ori_estimate_temp = f_ori2
            else:
                best_ori_estimate_temp = f_ori3

            # will get the best_ori_estimate in degrees , the choice is made so that any difference will be amplified more than radians
            best_ori_estimate = best_ori_estimate_temp
            print("best ori estimate: {} deg".format(m.degrees(best_ori_estimate)))
            # to debug lets plot the best_ori_estimate in the image
            pt1 = [200, 200]
            pt2 = [200, 200]
            line_angle = best_ori_estimate
            pt2[0] = int(pt2[0] + 200*m.cos(line_angle))
            pt2[1] = int(pt2[1] + 200*m.sin(line_angle))

            cv2.line(cv_image, (pt1[0], pt1[1]), (pt2[0], pt2[1]), (0, 0, 255), 3)

            # publish abs odometry for yaw
            # create euler angles
            roll = 0
            pitch = 0
            yaw = -best_ori_estimate

            # convert to quaternion
            q = quaternion_from_euler(roll, pitch, yaw)

            # create a odom message
            odom_msg = Odometry()
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.header.frame_id = "odom"
            odom_msg.header.stamp = rospy.Time().now()
            odom_pub.publish(odom_msg)

        rosimg = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(rosimg)

if __name__ == "__main__":
    try:
        abs_ori_sub = rospy.Subscriber("/stereo/left_upward/image_rect", Image, abs_ori_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
