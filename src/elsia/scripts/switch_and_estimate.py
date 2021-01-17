#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math as m
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import copy
import time

from heapq import heappush, heappop

rospy.init_node("switch_and_estimate_v002_node")
corrected_odom_pub = rospy.Publisher("/odom_sao", Odometry, queue_size=10)
raw_odom_pub = rospy.Publisher("/odom_rf2o_raw", Odometry, queue_size=10)
only_sao_odom_pub = rospy.Publisher("/only_sao_odom", Odometry, queue_size=10)

################################# Global Variables #############################
sao_x = 0.0
sao_y = 0.0
sao_yaw = 0.0  # SAO odometry

ceil_yaw = 0.0  # orientation obtained from ceiling

raw_x = 0.0
raw_y = 0.0
raw_yaw = 0.0  # raw odometry from laser using rf2o

# variable keeping track of whether we are inside a corridor or not.
isCorridor = False
# Initially set to False and will be updated from is_corridor_module.py
# variable keeping track of whether its the first time inside the aisle.
isEntryCorridor = True
# Set to True by default and will be set to False after the first time detection inside aisle and
# reset to True when coming out from a corridor.

block_array = []  # block array storing the detections inside the aisle [x,y, l_r]
# global variable keeping track of the block message, Initially set to
block_str_msg = None
# None, (Need to figure out how to reset this!)

H_aisle_to_glob = None
vel = 0.0
last_time_stamp = time.time()
odom_depth = None
ct_dist = None
# CAMERA PARAMS
f = 63.06
alpha = 0.264583
lat = 5.0
column_gap = 5.0


############################ Utility Functions #################################
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


def abs_odom_cb(msg):
    global ceil_yaw
    q = msg.pose.pose.orientation
    (_, _, ceil_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])


def isCorridor_cb(msg):
    global isCorridor
    string_data = msg.data
    if string_data == "True":
        isCorridor = True
    else:
        isCorridor = False


def cmd_vel_cb(msg):
    global vel
    vel = msg.linear.x
    omega = msg.angular.z


def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))


def aisle_ct_cb(msg):
    global ct_dist
    y = msg.pose.pose.position.y
    if ceil_yaw >= -m.pi/2 and ceil_yaw <= m.pi/2:
        ct_dist = -y
    else:
        ct_dist = y  # using positive value since our aisles are oriented at 180 degree


def publishOdom(xlsao, ylsao, yawlsao, xlraw, ylraw, yawlraw):
    # publish the odometry
    sao_odom_msg = Odometry()
    sao_q = quaternion_from_euler(0, 0, yawlsao)
    sao_odom_msg.pose.pose.position.x = xlsao
    sao_odom_msg.pose.pose.position.y = ylsao
    sao_odom_msg.pose.pose.orientation.x = sao_q[0]
    sao_odom_msg.pose.pose.orientation.y = sao_q[1]
    sao_odom_msg.pose.pose.orientation.z = sao_q[2]
    sao_odom_msg.pose.pose.orientation.w = sao_q[3]

    raw_odom_msg = Odometry()
    raw_q = quaternion_from_euler(0, 0, yawlraw)
    raw_odom_msg.pose.pose.position.x = xlraw
    raw_odom_msg.pose.pose.position.y = ylraw
    raw_odom_msg.pose.pose.orientation.x = raw_q[0]
    raw_odom_msg.pose.pose.orientation.y = raw_q[1]
    raw_odom_msg.pose.pose.orientation.z = raw_q[2]
    raw_odom_msg.pose.pose.orientation.w = raw_q[3]

    t = rospy.Time.now()

    sao_odom_msg.header.stamp = t
    # just adding frame_id and child_frame_id for sao odom
    # because that's all we care about in rviz, ignoring others 
    # right now
    sao_odom_msg.header.frame_id="/odom"
    sao_odom_msg.child_frame_id="/aisle_link"
    raw_odom_msg.header.stamp = t

    corrected_odom_pub.publish(sao_odom_msg)
    raw_odom_pub.publish(raw_odom_msg)


def publishonlySAOodom(xonlysao, yonlysao):
    msg = Odometry()
    msg.pose.pose.position.x = xonlysao
    msg.pose.pose.position.y = yonlysao
    only_sao_odom_pub.publish(msg)
################################################################################

######################## SAO related functions #################################


def ignoreErratic(odom_depth_1, odom_depth_0):
    if abs(odom_depth - odom_depth_1) > 2:
        odom_depth_loc = odom_depth_0
    else:
        odom_depth_loc = odom_depth_1
    return odom_depth_loc


def getDepth(l_r, px):
    camera_front_offset = 0.1
    # the aisle yaw would depend on which direction the robot is entering from
    if ceil_yaw >= -m.pi/2 and ceil_yaw <= m.pi/2:
        aisle_yaw = ceil_yaw
    else:
        aisle_yaw = wrapToPi(ceil_yaw - m.pi)

    # print("aisle_yaw is: {}".format(m.degrees(aisle_yaw)))
    ct_camera = ct_dist - (camera_front_offset) * m.sin(aisle_yaw)
    # print("ct_dist is : {} and ct_camera is: {}".format(ct_dist, ct_camera))
    if l_r == "l":
        # print("it is l")
        # find xa and ya
        xa = px * (alpha/1000) * m.cos(aisle_yaw) - \
            (f/1000) * m.sin(aisle_yaw) + ct_camera
        ya = px * (alpha/1000) * m.sin(aisle_yaw) + (f/1000) * m.cos(aisle_yaw)
        depth = ((-(lat/2) - ct_camera) * ya) / (xa - ct_camera)
    else:
        # find xa and ya
        xa = px * (alpha/1000) * m.cos(aisle_yaw) - \
            (f/1000) * m.sin(aisle_yaw) + ct_camera
        ya = px * (alpha/1000) * m.sin(aisle_yaw) + (f/1000) * m.cos(aisle_yaw)
        # print("it is r")
        depth = (((lat/2) - ct_camera) * ya) / (xa - ct_camera)

    depth = depth + (camera_front_offset) * m.cos(aisle_yaw)
    # print("depth value for abs_yaw = {}, ct_camera = {}, px = {} is: {}".format(aisle_yaw, ct_camera, px, depth))
    return depth


def getRawDepthInAisle():
    if block_str_msg == None:
        return None
    block_str_msg_temp = copy.deepcopy(block_str_msg)
    data = block_str_msg_temp.split(",")

    block_count = float(data[0])
    px = float(data[1])  # in pixels which is basically the x value
    l_r = data[2]
    px0 = float(data[3])  # second last value
    l_r0 = data[4]  # second last value

    # print("original px was: {}".format(px))
    px = px - 200
    px0 = px0 - 200

    depth = getDepth(l_r, px)
    depth0 = getDepth(l_r0, px0)

    # print("depth value is : {}".format(depth))

    odom_depth_1 = (block_count * column_gap) - depth
    odom_depth_0 = ((block_count - 1) * column_gap) - depth0

    if odom_depth is not None:
        odom_depth_final = ignoreErratic(odom_depth_1, odom_depth_0)
    else:
        odom_depth_final = odom_depth_1

    return odom_depth_final


def update_blocks(cv_image, x, y, l_r):
    global block_array, block_str_msg

    if len(block_array) == 0:
        block_array.append([x, y, l_r])
        # print("y value is: {} and block array is : {}".format(y, block_array))
        msg = str(len(block_array)) + "," + str(block_array[-1][0]) + "," + block_array[-1][2] + "," + str(
            block_array[-1][0]) + "," + block_array[-1][2]  # (block_count, x1, l_r1, x0, l_r0)

        prev = (0, int(y))
        next = (400, int(y))
        cv2.line(cv_image, prev, next, (0, 255, 255), 2)
        # also plot the vertical line
        cv2.line(cv_image, (int(block_array[-1][0]), 0),
                 (int(block_array[-1][0]), 400), (125, 0, 125), 2)

        # update global value
        block_str_msg = msg
        # block_pub.publish(msg)
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

    # print("y value is: {} and block array is : {}".format(y, block_array))
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

    # update global value
    block_str_msg = msg
    # block_pub.publish(msg)
    return cv_image


def img_cb(msg):
    global block_array, block_str_msg
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print("[WARN]: Error in obtaining image from CvBridge! Skipping frame!")
    else:
        # convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # threshold image
        ret, thresh2 = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # find edges
        edges = cv2.Canny(gray, 80, 120)

        """
        Lane line detection
        """
        mask = np.zeros_like(gray)
        # mask[int((2*mask.shape[1])/3) :, :] = 255
        mask[195:, :] = 255
        masked_img = cv2.bitwise_and(edges, edges, mask=mask)
        thresh2_masked = cv2.bitwise_and(thresh2, thresh2, mask=mask)
        lines_angled = cv2.HoughLines(masked_img, 1, m.pi/180, 100)

        line_factor = 500
        left_theta = []
        right_theta = []
        left_rho = []
        right_rho = []
        left_theta_avg = None
        right_theta_avg = None
        left_rho_avg = None
        right_rho_avg = None

        if lines_angled is None:
            block_str_msg = None

        if lines_angled is not None:
            for i in range(lines_angled.shape[0]):
                for rho, theta in lines_angled[i]:
                    # print(rho, theta)
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
        lines = cv2.HoughLinesP(thresh2_masked, 1, m.pi/2, 2, None, 45, 1)

        if lines is None:
            block_str_msg = None

        if lines is not None and lines_angled is not None:
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

            if len(lines_drawn) != 0:
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
                consider_thresh = 20  # 7

                if left_theta_avg != None:
                    slop_left = m.tan(m.pi/2 + left_theta_avg)
                    intercept_left = - \
                        (slop_left * left_rho_avg) / m.cos(left_theta_avg)
                if right_theta_avg != None:
                    slop_right = m.tan(m.pi/2 + right_theta_avg)
                    intercept_right = - \
                        (slop_right * right_rho_avg) / m.cos(right_theta_avg)

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
                                cv2.line(cv_image, (int(run_x_avg), 0), (int(
                                    run_x_avg), int(run_y)), (255, 0, 255), 2)
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
                                cv2.line(cv_image, (int(run_x_avg), 0), (int(
                                    run_x_avg), int(run_y)), (255, 0, 255), 2)
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

                    """ This is for plotting the yellow line connecting ends of  the vertical pink lines"""
                    for i in range(0, 1):
                        cur = pts_list[i]
                        prev = (0, int(cur[1]))
                        next = (400, int(cur[1]))
                        # for plotting line with recently acquired information
                        cv2.line(cv_image, prev, next, (255, 0, 0), 2)
                        if isEntryCorridor:
                            block_array = []
                        if isCorridor:
                            cv_image = update_blocks(
                                cv_image, cur[0], cur[1], cur[2])

                # lets also put text about current block length info to debug
                if isCorridor:
                    # if block_str_msg is None:
                    # print("ALERT block_str_msg is None!")
                    # pass
                    cv_image = puttext(cv_image, str(len(block_array)))
                    cv2.namedWindow("switch_and_estimate_node",
                                    cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("switch_and_estimate_node", 500, 500)
                    cv2.imshow("switch_and_estimate_node", cv_image)
                    cv2.imshow("masked_img", masked_img)
                    cv2.imshow("thresholded_masked", thresh2_masked)
                    # cv2.imshow("masked", masked_img)
                    cv2.waitKey(1)


printOnce = True


def laser_odom_cb(msg):
    global printOnce
    global sao_x, sao_y, sao_yaw, raw_x, raw_y, raw_yaw
    global isCorridor, isEntryCorridor
    global block_array
    global H_aisle_to_glob
    global odom_depth, block_str_msg
    ################# motion model related logic and variables #################
    global last_time_stamp

    cur_time_stamp = time.time()
    delta_time = cur_time_stamp - last_time_stamp
    last_time_stamp = cur_time_stamp
    delta_x_motion = vel * delta_time * m.cos(ceil_yaw)
    ############################################################################
    # extract the pose
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # find the delta information
    line_angle = m.atan2((y - raw_y), (x - raw_x))
    beta = wrapToPi(line_angle - raw_yaw)
    d = m.sqrt((x - raw_x)**2 + (y - raw_y)**2)
    delta_x_loc = d * m.cos(beta)
    delta_y_loc = d * m.sin(beta)

    # reset the raw values
    raw_x = x
    raw_y = y
    raw_yaw = yaw

    # convert the local deltas to global deltas
    delta_x_g = delta_x_loc * m.cos(sao_yaw) - delta_y_loc * m.sin(sao_yaw)
    delta_y_g = delta_x_loc * m.sin(sao_yaw) + delta_y_loc * m.cos(sao_yaw)

    if isCorridor:
        if isEntryCorridor:
            # set isEntryCorridor to False
            isEntryCorridor = False

        if block_str_msg is None:
            print("BLOCK NONE!!!!!!!!!!!")
            # we dont have any value to get the depth
            # hence we need to trust the corrected laser odometry
            sao_x = sao_x + delta_x_g
            sao_y = sao_y + delta_y_g
            sao_yaw = ceil_yaw
            publishOdom(sao_x, sao_y, sao_yaw, raw_x, raw_y, raw_yaw)
            return
        else:
            print("sao usage")
            # find the aisle cordinates
            a_x = ct_dist
            a_y = getRawDepthInAisle()

            if ceil_yaw >= -m.pi/2 and ceil_yaw <= m.pi/2:
                a_yaw = wrapToPi(ceil_yaw + m.pi/2)
            else:
                a_yaw = wrapToPi(ceil_yaw - m.pi/2)

            # print("a_x: {}, a_y: {}, a_yaw:{}deg before transformation".format(a_x, a_y, m.degrees(a_yaw)))
            if odom_depth is None:
                odom_depth = a_y
            else:
                weight = 0.0  # 0.98
                a_y = (1 - weight) * a_y + weight * \
                    (odom_depth + delta_x_motion)
                odom_depth = a_y

            if H_aisle_to_glob is None:
                H_sao_to_aisle = np.array([[m.cos(a_yaw), -m.sin(a_yaw), a_x],
                                           [m.sin(a_yaw),  m.cos(a_yaw), a_y],
                                           [0,             0,   1]])

                # print("H_sao_to_aisle: \n{}\n".format(H_sao_to_aisle))
                H_aisle_to_sao = np.linalg.inv(H_sao_to_aisle)
                # print("H_aisle_to_sao: \n{}\n".format(H_aisle_to_sao))

                H_sao_to_glob = np.array([[m.cos(ceil_yaw), -m.sin(ceil_yaw), sao_x + delta_x_g],
                                          [m.sin(ceil_yaw),  m.cos(
                                              ceil_yaw), sao_y + delta_y_g],
                                          [0,                   0,                 1]])

                # print("H_sao_to_glob: \n{}\n".format(H_sao_to_glob))
                H_aisle_to_glob = np.dot(H_sao_to_glob, H_aisle_to_sao)
                # print("H_aisle_to_glob after (H_sao_to_glob X H_aisle_to_sao) : {}".format(H_aisle_to_glob))
                # print("raw_x: {}, raw_y: {}, raw_yaw: {}".format(raw_x, raw_y, m.degrees(raw_yaw)))

                # verification
                A = np.dot(H_aisle_to_glob, H_sao_to_aisle)
                # print("A: \n{}\n should be same as H_sao_to_glob: \n{}\n".format(A, H_sao_to_glob))

            H_sao_to_aisle = np.array([[m.cos(a_yaw), -m.sin(a_yaw), a_x],
                                       [m.sin(a_yaw),  m.cos(a_yaw), a_y],
                                       [0,             0,   1]])
            # A = H *B
            # where A =  H_sao_to_glob ; H = H_aisle_to_glob; B = H_sao_to_aisle
            A = np.dot(H_aisle_to_glob, H_sao_to_aisle)
            sao_x = float(A[0][2])
            sao_y = float(A[1][2])
            sao_yaw = ceil_yaw

            publishonlySAOodom(sao_x, sao_y)

            # check if this values are pretty far off
            # rejectThresh = 0.2
            # if abs(sao_x_temp - sao_x) > 0.2:
            #     sao_x = sao_x + delta_x_motion
            #     sao_y = sao_y_temp

            # if printOnce:
            # print("sao_x: {}, sao_y: {}, sao_yaw:{}deg after transformation".format(sao_x, sao_y, m.degrees(sao_yaw)))
            printOnce = False
            # print("raw_x: {}, raw_y: {}, raw_yaw:{}deg".format(raw_x, raw_y, m.degrees(raw_yaw)))
    else:
        # reset values
        odom_depth = None
        block_str_msg = None
        H_aisle_to_glob = None
        isEntryCorridor = True
        # use the delta values from laser to update sao values
        sao_x = sao_x + delta_x_g
        sao_y = sao_y + delta_y_g
        sao_yaw = ceil_yaw
        # print("[NORMAL]: sao_x: {}, sao_y: {}, sao_yaw:{}deg after transformation".format(sao_x, sao_y, m.degrees(sao_yaw)))

    publishOdom(sao_x, sao_y, sao_yaw, raw_x, raw_y, raw_yaw)

################################################################################


############################### Main ###########################################
if __name__ == "__main__":
    try:
        rf2o_sub = rospy.Subscriber("/odom_rf2o", Odometry, laser_odom_cb)
        abs_odom_sub = rospy.Subscriber(
            "/abs_orientation_odom", Odometry, abs_odom_cb)
        isCorridor_sub = rospy.Subscriber(
            "/is_corridor", String, isCorridor_cb)
        # using the aisle cross track
        aisle_ct_sub = rospy.Subscriber("/aisle_ct", Odometry, aisle_ct_cb)
        cmd_vel_sub = rospy.Subscriber("/jacky/cmd_vel", Twist, cmd_vel_cb)
        img_sub = rospy.Subscriber("/stereo/left/image_rect", Image, img_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
################################################################################
