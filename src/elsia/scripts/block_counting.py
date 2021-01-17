#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math as m

# initialize the node
rospy.init_node("block_counting_act_prop_node")

block_pub = rospy.Publisher("/block_val", String, queue_size=10)

sav_cnt = 0
# it now has to be a 3 tuple [x, y, l/r](actually a list, because mutable)
block_array = []
global_x = 0


def update_blocks(cv_image, x, y, l_r):
    global block_array

    if len(block_array) == 0:
        block_array.append([x, y, l_r])
        print("y value is: {} and block array is : {}".format(y, block_array))
        msg = str(len(block_array)) + "," + str(block_array[-1][0]) + "," + block_array[-1][2] + "," + str(
            block_array[-1][0]) + "," + block_array[-1][2]  # (block_count, x1, l_r1, x0, l_r0)
        prev = (0, int(y))
        next = (400, int(y))
        cv2.line(cv_image, prev, next, (0, 255, 255), 2)
        # also plot the vertical line
        cv2.line(cv_image, (int(block_array[-1][0]), 0),
                 (int(block_array[-1][0]), 400), (125, 0, 125), 2)
        block_pub.publish(msg)
        return cv_image
    # otherwise find the one to update or append new
    thresh = 40  # 20#15#10 # in pixel
    if (block_array[-1][1] - y) > thresh:
        # this suggests a new block and hence append
        block_array.append([x, y, l_r])
    elif abs(y - block_array[-1][1]) < thresh:
        # update the last entry
        block_array[-1] = [x, y, l_r]
    else:
        # back jump
        # update the second last entry
        block_array[-2] = [x, y, l_r]

    print("y value is: {} and block array is : {}".format(y, block_array))
    if len(block_array) >= 2:
        msg = str(len(block_array)) + "," + str(block_array[-1][0]) + "," + block_array[-1][2] + "," + str(
            block_array[-2][0]) + "," + block_array[-2][2]  # (block_count, x1, l_r1, x0, l_r0)
    else:
        msg = str(len(block_array)) + "," + str(block_array[-1][0]) + "," + block_array[-1][2] + "," + str(
            block_array[-1][0]) + "," + block_array[-1][2]  # (block_count, x1, l_r1, x0, l_r0)

    prev = (0, int(block_array[-1][1]))
    next = (400, int(block_array[-1][1]))
    cv2.line(cv_image, prev, next, (0, 255, 255), 2)
    # also plot the vertical line
    cv2.line(cv_image, (int(block_array[-1][0]), 0),
             (int(block_array[-1][0]), 400), (125, 0, 125), 2)

    block_pub.publish(msg)
    return cv_image


def puttext(img, text):
    # text properties
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50, 50)
    fontScale = 0.5
    color = (255, 0, 0)
    thickness = 2

    img = cv2.putText(img, text, org, font, fontScale,
                      color, thickness, cv2.LINE_AA)
    return img


def find_contours(img, edges):
    _, contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img, contours, -1, (255, 255, 255), 2)
    return img


def img_cb(msg):
    global sav_cnt, block_array
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print("[INFO]: Error in obtaining image from CvBridge! Skipping frame!")
    else:
        # converting to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # threshold image
        ret, thresh1 = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        ret, thresh2 = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # apply erosion to thresholded image
        kernel = np.ones((3, 3), np.uint8)
        eroded = cv2.erode(thresh2, kernel, iterations=1)

        # find edges
        edges = cv2.Canny(gray, 80, 120)

        """
        Lane line detection
        """
        contoured_img = find_contours(cv_image.copy(), edges.copy())

        # use contoured img to get the lines in bottom half using HoughLines and not HoughLinesP
        contoured_img_gray = cv2.cvtColor(contoured_img, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(contoured_img_gray)
        mask[int((2*mask.shape[1])/3):, :] = 255
        masked_img = cv2.bitwise_and(edges, edges, mask=mask)
        lines_angled = cv2.HoughLines(masked_img, 1, m.pi/180, 75)

        line_factor = 500
        left_theta = []
        right_theta = []
        left_rho = []
        right_rho = []
        left_theta_avg = None
        right_theta_avg = None
        left_rho_avg = None
        right_rho_avg = None

        if lines_angled.shape[0] != 0:
            for i in range(lines_angled.shape[0]):
                for rho, theta in lines_angled[i]:
                    print(rho, theta)
                    if (theta) < m.radians(5):
                        continue
                    if theta < m.pi/2:
                        left_theta.append(theta)
                        left_rho.append(rho)
                    else:
                        right_theta.append(theta)
                        right_rho.append(rho)

            if len(left_theta) != 0:
                left_theta_avg = sum(left_theta) / len(left_theta)
                left_rho_avg = sum(left_rho) / len(left_rho)
                a = np.cos(left_theta_avg)
                b = np.sin(left_theta_avg)
                x0 = a*left_rho_avg
                y0 = b*left_rho_avg
                x1 = int(x0 + line_factor*(-b))
                y1 = int(y0 + line_factor*(a))
                x2 = int(x0 - line_factor*(-b))
                y2 = int(y0 - line_factor*(a))

                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

            if len(right_theta) != 0:
                right_theta_avg = sum(right_theta) / len(right_theta)
                right_rho_avg = sum(right_rho) / len(right_rho)
                a = np.cos(right_theta_avg)
                b = np.sin(right_theta_avg)
                x0 = a*right_rho_avg
                y0 = b*right_rho_avg
                x1 = int(x0 + line_factor*(-b))
                y1 = int(y0 + line_factor*(a))
                x2 = int(x0 - line_factor*(-b))
                y2 = int(y0 - line_factor*(a))

                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 125, 125), 2)

        """
        vertical lines detection
        """
        # extract HoughP lines
        lines = cv2.HoughLinesP(thresh2, 1, m.pi/2, 2, None, 70, 1)

        lines_drawn = []
        for i in range(lines.shape[0]):
            for line in lines[i]:
                pt1 = (line[0], line[1])
                pt2 = (line[2], line[3])
                # to just detect vertical lines
                if (line[0] != line[2]):
                    continue
                # to remove the ceiling lines
                max_ordinate = max(line[1], line[3])
                if max_ordinate < 200:
                    continue
                """ to plot blue lines """
                cv2.line(cv_image, pt1, pt2, (255, 0, 0), 3)
                lines_drawn.append(pt1)
                lines_drawn.append(pt2)

        # sort the lines_drawn pts
        lines_drawn.sort(key=lambda x: x[0])
        # print("{} vertical lines were detected: and they are \n {}".format(len(lines_drawn), lines_drawn))
        lines_drawn_new = [n for n in lines_drawn if n[1] > 200]
        # print("new points are : \n {}".format(lines_drawn_new))

        # lets use bunching up techniques
        run_x_avg = lines_drawn_new[0][0]
        run_y = lines_drawn_new[0][1]
        pts_list = []
        count = 1
        x_thresh = 10
        consider = False
        consider_thresh = 7

        if left_theta_avg != None:
            slop_left = m.tan(m.pi/2 + left_theta_avg)
            intercept_left = -(slop_left * left_rho_avg) / \
                m.cos(left_theta_avg)
        if right_theta_avg != None:
            slop_right = m.tan(m.pi/2 + right_theta_avg)
            intercept_right = -(slop_right * right_rho_avg) / \
                m.cos(right_theta_avg)

        for i in range(1, len(lines_drawn_new)):
            if abs(run_x_avg - lines_drawn_new[i][0]) <= x_thresh:
                count += 1
                # calculate the new x avg
                run_x_avg = run_x_avg + \
                    ((lines_drawn_new[i][0] - run_x_avg)*1.0) / count
                run_y = max(run_y, lines_drawn_new[i][1])
            else:
                # we need to check if this point is close enough to the intersection point with either of the lane lines
                consider = False
                if left_theta_avg != None:
                    y_intersect = slop_left * run_x_avg + intercept_left
                    if abs(y_intersect - run_y) <= consider_thresh:
                        consider = True
                        run_y = y_intersect
                        pts_list.append([run_x_avg, run_y, "l"])
                        """ To plot the magenta lines """
                        cv2.line(cv_image, (int(run_x_avg), 0),
                                 (int(run_x_avg), int(run_y)), (255, 0, 255), 2)
                        run_x_avg = lines_drawn_new[i][0]
                        run_y = lines_drawn_new[i][1]
                        count = 1
                if consider == False and right_theta_avg != None:
                    y_intersect = slop_right * run_x_avg + intercept_right
                    if abs(y_intersect - run_y) <= consider_thresh:
                        consider = True
                        run_y = y_intersect
                        pts_list.append([run_x_avg, run_y, "r"])
                        """ To plot the magenta lines """
                        cv2.line(cv_image, (int(run_x_avg), 0),
                                 (int(run_x_avg), int(run_y)), (255, 0, 255), 2)
                        run_x_avg = lines_drawn_new[i][0]
                        run_y = lines_drawn_new[i][1]
                        count = 1

        consider = False
        if left_theta_avg != None:
            y_intersect = slop_left * run_x_avg + intercept_left
            if abs(y_intersect - run_y) <= consider_thresh:
                consider = True
                run_y = y_intersect
                pts_list.append([run_x_avg, run_y, "l"])
                """ To plot the magenta lines """
                cv2.line(cv_image, (int(run_x_avg), 0),
                         (int(run_x_avg), int(run_y)), (255, 0, 255), 2)

        if consider == False and right_theta_avg != None:
            y_intersect = slop_right * run_x_avg + intercept_right
            if abs(y_intersect - run_y) <= consider_thresh:
                consider = True
                run_y = y_intersect
                pts_list.append([run_x_avg, run_y, "r"])
                """ To plot the magenta lines """
                cv2.line(cv_image, (int(run_x_avg), 0),
                         (int(run_x_avg), int(run_y)), (255, 0, 255), 2)

        if len(pts_list) != 0:
            # sort this pts_list based on second index
            pts_list.sort(key=lambda x: x[1], reverse=True)

            prev = pts_list[0]
            # for i in range(1, len(pts_list)):
            #     cur = pts_list[i]
            #     if abs(cur[1] - prev[1]) <= 2:
            #         # draw a line
            #         cv2.line(cv_image, (prev[0], prev[1]), (cur[0], cur[1]), (0, 100, 100), 2)
            #     prev = cur

            """ This is for plotting the yellow line connecting ends of  the vertical pink lines"""
            for i in range(0, 1):
                cur = pts_list[i]
                prev = (0, int(cur[1]))
                next = (400, int(cur[1]))
                # for plotting line with recently acquired information
                cv2.line(cv_image, prev, next, (255, 0, 0), 2)
                cv_image = update_blocks(cv_image, cur[0], cur[1], cur[2])

        # lets also put text about current block length info to debug
        cv_image = puttext(cv_image, str(len(block_array)))

        cv2.namedWindow("HoughLines", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HoughLines", 800, 800)
        cv2.imshow("HoughLines", cv_image)
        cv2.imshow("masked", masked_img)
        cv2.waitKey(1)


def odom_cb(odom_msg):
    global global_x
    pose = odom_msg.pose.pose
    global_x = pose.position.x


if __name__ == "__main__":
    try:
        img_sub = rospy.Subscriber("/stereo/left/image_rect", Image, img_cb)
        pose_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
        rospy.spin()
        cv2.destroyAllWindows()

        # print number of blocks traversed
        print("Number of blocks traversed are : {} and a coarse longitudinal estimate is : {} and the variance estimates are: [{}, {}]".format(
            len(block_array)-1, (len(block_array)-1)*8, (len(block_array)-1)*8 - 8, (len(block_array)-1)*8 + 8))
        # print the x of the ground truth
        print("Length according to ground truth is: {}".format(global_x))
    except rospy.ROSInterruptException:
        pass
