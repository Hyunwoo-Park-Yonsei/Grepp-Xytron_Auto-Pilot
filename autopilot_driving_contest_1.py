#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math, time, collections
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from PID import PID
import sys
import os
import signal


def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 280
Gap = 36


def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")


# publish xycar_motor msg
def drive(Angle, Speed):
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)


def ROI(frame, vertices):
    # blank mask:
    mask = np.zeros_like(frame)
    # fill the mask
    cv2.fillPoly(mask, vertices, 255)

    # now only show the area that is the mask
    masked = cv2.bitwise_and(frame, mask)
    return masked


# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1 + Offset), (x2, y2 + Offset), color, 2)
    return img


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                  (lpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                  (rpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (center - 5, 15 + offset),
                  (center + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset),
                  (325, 25 + offset),
                  (0, 0, 255), 2)
    return img


# left lines, right lines
def divide_left_right_curve(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width / 2 - 40):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width / 2 + 40):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines


def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width / 2 - 130):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width / 2 + 130):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

def center_line(lines):
    global Width

    # calculate slope & filtering with threshold
    new_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
    
            if -0.09 < (x2 - x1) < 0.09:
                slope = 0
                new_lines.append(line[0])
    return new_lines


def divide_left(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    new_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
    
            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2 - y1) / float(x2 - x1)
    
            if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
                new_lines.append(line[0])

    # divide lines left to right
    left_lines = []

    for j in range(len(new_lines)):
        Line = new_lines[j]

        x1, y1, x2, y2 = Line

        if (slope < 0): #and (x2 < Width / 2 - 130):
            left_lines.append([Line.tolist()])

    return left_lines

def divide_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    new_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
    
            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2 - y1) / float(x2 - x1)
    
            if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
                new_lines.append(line[0])

    # divide lines left to right
    right_lines = []

    for j in range(len(new_lines)):
        Line = new_lines[j]

        x1, y1, x2, y2 = Line

        if (slope > 0): #and (x1 > Width / 2 + 130):
            right_lines.append([Line.tolist()])

    return right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b


# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap/2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height / 2) - b) / float(m)

        #cv2.line(img, (int(x1), Height), (int(x2), (Height / 2)), (255, 0, 0), 3)

    return img, pos, m


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global curve

    # gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # ROI
    # if curve:
    
    #roi_center = edge_img[260 : 290, 302 : 328]
    
    roi_left = edge_img[240 : 270, 200 : 240]
    roi_right = edge_img[240 : 270, 400 : 440]

    if curve:
        #vertices1 = np.array([[(0, Gap), (20, 0), (Width / 2 - 10, 0), (Width / 2 - 120, Gap)]], dtype=np.int32)
        #vertices2 = np.array([[(Width / 2 + 120, Gap), (Width / 2 + 10, 0), (Width-20, 0), (Width, Gap)]], dtype=np.int32)
        vertices = np.array([[(5, Gap), (30, 0), (Width - 30, 0), (Width-5, Gap)]],
                             dtype=np.int32)
        roi = edge_img[Offset: Offset + Gap, 0: Width]
        roi = ROI(roi, [vertices])
        #roi_frame1 = ROI(roi, [vertices1])
        #roi_frame2 = ROI(roi, [vertices2])
        #roi_frame = cv2.add(roi_frame1, roi_frame2)
        #roi = np.uint8(roi_frame)
    else:
        vertices1 = np.array([[(10, Gap), (125, 0), (160, 0), (180, Gap)]], dtype=np.int32)
        vertices2 = np.array([[(Width - 190, Gap), (Width - 170, 0), (Width - 130, 0), (Width-10, Gap)]], dtype=np.int32)
        roi = edge_img[Offset: Offset + Gap, 0: Width]
        roi_frame1 = ROI(roi, [vertices1])
        roi_frame2 = ROI(roi, [vertices2])
        roi_frame = cv2.add(roi_frame1, roi_frame2)
        roi = np.uint8(roi_frame)

    #cv2.imshow('roi', roi)
    #cv2.imshow('roi_left', roi_left)
    #cv2.imshow('roi_right', roi_right)
    # HoughLinesP
    all_lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, 30, 10)
    center_left = cv2.HoughLinesP(roi_left, 1, math.pi / 180, 10, 10, 5)
    center_right = cv2.HoughLinesP(roi_right, 1, math.pi / 180, 10, 10, 5)
    center_left_1 = divide_left(center_left)
    center_right_2 = divide_right(center_right)
    #print('center:', left_line_center, right_line_center)
    #print(center_left_1, center_right_2)
    global speed_slow
    if not center_left_1 or not center_right_2:
        speed_slow = True
    else:
        speed_slow = False
   

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    if curve:
        left_lines, right_lines = divide_left_right_curve(all_lines)
    else:
        left_lines, right_lines = divide_left_right(all_lines)
    # get center of lines
    frame, lpos, m_left = get_line_pos(frame, left_lines, left=True)
    frame, rpos, m_right = get_line_pos(frame, right_lines, right=True)
    

    if -0.42 > m_left or m_left > -0.28:
        curve = True
    elif 0.27 > m_right or m_right > 0.44:
        curve = True
    else:
        curve = False

    line_width = rpos - lpos
    global line_width_avg, line_width_deque, lpos_avg, lpos_deque, rpos_avg, rpos_deque

    if len(line_width_deque) >= 5:
        line_width_avg_a = (line_width_deque[0] + 2 * line_width_deque[1] + 3 * line_width_deque[2] + 4 *
                          line_width_deque[3] + 5 * line_width_deque[4]) / 15
        line_width_deque.popleft()
    else:
        line_width_avg_a = line_width
    line_width_deque.append(line_width)
    #print('line_width:', line_width)
    #print('line_width_avg:', line_width_avg)
    
    if curve:
        line_width_avg = line_width_avg_a
    else:
        line_width_avg = line_width_avg_a #485
    
    if line_width <= line_width_avg - 7 or line_width >= line_width_avg + 7:
        if lpos_avg - 8 < lpos < lpos_avg + 8:
            rpos = rpos_avg
        elif rpos_avg - 8 < rpos < rpos_avg + 8:
            lpos = lpos_avg

    #print('rpos:', rpos)
    lpos_deque.append(lpos)
    if len(lpos_deque) >= 5:
        lpos_avg = (lpos_deque[0] + 2 * lpos_deque[1] + 3 * lpos_deque[2] + 4 * lpos_deque[3] + 5 * lpos_deque[4]) / 15
        # if lpos > lpos_avg +15:
        #    lpos = lpos_avg
        # elif lpos == 0:
        #    lpos = lpos_avg
        lpos = lpos_avg
        lpos_deque.popleft()
    else:
        lpos_avg = lpos
    # lpos_deque.append(lpos)

    rpos_deque.append(rpos)
    if len(rpos_deque) >= 5:
        rpos_avg = (rpos_deque[0] + 2 * rpos_deque[1] + 3 * rpos_deque[2] + 4 * rpos_deque[3] + 5 * rpos_deque[4]) / 15
        # if rpos < rpos_avg -15:
        #    rpos = rpos_avg
        # elif rpos == 0:
        #    rpos = rpos_avg
        rpos = rpos_avg
        rpos_deque.popleft()
    else:
        rpos_avg = rpos
    # rpos_deque.append(rpos)

    # draw lines
    #frame = draw_lines(frame, left_lines)
    #frame = draw_lines(frame, right_lines)
    #frame = cv2.line(frame, (230, 235), (410, 235), (255, 255, 255), 2)

    # draw rectangle
    #frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    # roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    # roi2 = draw_rectangle(roi2, lpos, rpos)

    # show image
    # cv2.imshow('calibration', frame)

    return lpos, rpos


def start():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    # cap = cv2.VideoCapture('kmu_track.mkv')
    # cap = cv2.VideoCapture('track1.avi')
    # cap = cv2.VideoCapture('track2.avi')

    print
    "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(0.5)

    while True:
        while not image.size == (640 * 480 * 3):
            continue
        #start = time.time()  #
        # ret, image = cap.read()
        lpos, rpos = process_image(image)

        center = (lpos + rpos) / 2
        angle = -(Width / 2 - center)

        if curve:
            speed = 18
            pid = PID(0.175, 0.0006, 0.055)
            pid_angle = pid.pid_control(angle) 
        elif speed_slow:
            speed = 25
            pid = PID(0.045, 0.00035, 0.04)
            pid_angle = pid.pid_control(angle)
        else:
            speed = 50
            pid = PID(0.015, 0.0001, 0.025)
            pid_angle = pid.pid_control(angle)
      
        drive(pid_angle, speed)
        #cv2.imshow('x', image)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
        #print('time:', time.time() - start)
        # time.sleep(0.03333)

    rospy.spin()


if __name__ == '__main__':
    curve = True
    speed_slow = False
    line_width_avg = 485
    line_width_deque = collections.deque([])
    lpos_avg = 565-485
    lpos_deque = collections.deque([])
    rpos_avg = 565
    rpos_deque = collections.deque([])
    start()
