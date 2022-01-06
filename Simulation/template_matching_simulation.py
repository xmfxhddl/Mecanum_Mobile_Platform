#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sim
import sympy as sp
import numpy as np
import time
import array
import math
import imutils
from func import *
from roi_point_func import *
from PIL import Image as I
from skimage.transform import (hough_line, hough_line_peaks)
from cv2 import cv2
import pickle

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
##########

real_robot_x = []
real_robot_y = []

time_list = [0] 

##########

n = 0
k = 0
o1 = 0
t = 0
###########

kalman_u = 0
kalman_k = 0

real_x = []
real_y = []

real_x_k = []
real_y_k = []

kalman_x = []
kalman_y = []

kalman_x_k = [2633, 2633]
kalman_y_k = [2663, 2663]

kalman_measure_x = [2633, 2633]
kalman_measure_y = [2663, 2663]

kalman_measure_x_k = [2633, 2633]
kalman_measure_y_k = [2663, 2663]

#### plt ###

error_kalman_x = [0]
error_kalman_y = [0]

error_kalman_x_k = [0]
error_kalman_y_k = [0]

error_x = [0]
error_y = [0]

###########

initial_x1 = [1898]
initial_y1 = [2661]
initial_x2 = [3352]
initial_y2 = [2659]
initial_x3 = [2663]
initial_y3 = [1936]
initial_x4 = [2633]
initial_y4 = [3390]

degree_list = [0, 0]

robot_x = [2633]
robot_y = [2663]

list_angle = [0, 0]

measure_robot_x = [2633, 2633]
measure_robot_y = [2663, 2663]

##########
x_0 = np.array([2633, 0, 2664, 0])  # (x-pos, x-vel, y-pos, y-vel) by definition in book.
P_0 = 100 * np.eye(4)

x_esti, P = None, None

###########
#### connect gpu ####

#print(cv2.cuda.getCudaEnabledDeviceCount()) # 0=안함 1=작동

#### connect v-rep ####

clientID = connect(20210125)

#### remote API ####

# vision sensor
res1, cam1 = sim.simxGetObjectHandle(clientID, '0_8m_35(131)_left', sim.simx_opmode_oneshot_wait)
res2, cam2 = sim.simxGetObjectHandle(clientID, '0_8m_35(131)_right', sim.simx_opmode_oneshot_wait)
res3, cam3 = sim.simxGetObjectHandle(clientID, '0_8m_35(131)_front', sim.simx_opmode_oneshot_wait)
res4, cam4 = sim.simxGetObjectHandle(clientID, '0_8m_35(131)_back', sim.simx_opmode_oneshot_wait)

err1, resolution1, image1 = sim.simxGetVisionSensorImage(clientID, cam1, 0, sim.simx_opmode_streaming)
err2, resolution2, image2 = sim.simxGetVisionSensorImage(clientID, cam2, 0, sim.simx_opmode_streaming)
err3, resolution3, image3 = sim.simxGetVisionSensorImage(clientID, cam3, 0, sim.simx_opmode_streaming)
err4, resolution4, image4 = sim.simxGetVisionSensorImage(clientID, cam4, 0, sim.simx_opmode_streaming)

# mecanum wheel orientation
coin, robot = sim.simxGetObjectHandle(clientID, 'swim', sim.simx_opmode_oneshot_wait)

ret, center_orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
ret1, center_position = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
ret2, center_position1 = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)

#### perspective transformation matrix ####
src = np.array([[475, 39], [804, 39],[0, 315],[1280, 315]], np.float32)

dst = np.array([[0,0],[476,0],[0,550],[476,550]], np.float32)

M = cv2.getPerspectiveTransform(src, dst)

print(M)

path_image = cv2.imread('image/blue_print/court_test_change_white.png', cv2.IMREAD_COLOR)
kalman_image = cv2.imread('image/blue_print/court_test_change_white.png', cv2.IMREAD_COLOR)
roro_image = cv2.imread('image/blue_print/court_test_change_white.png', cv2.IMREAD_COLOR)

real_moving_image = cv2.imread('image/blue_print/court_test_change_white.png', cv2.IMREAD_COLOR)
real_moving_image_k = cv2.imread('image/blue_print/court_test_change_white.png', cv2.IMREAD_COLOR)

height = path_image.shape[0]
width = path_image.shape[1]

height1 = int(height) / 2
width1 = int(width) / 2

# ##### main #####

# while (sim.simxGetConnectionId(clientID) != -1):
    
#     start = time.time()
#     second = time.time()
#     third = time.time()

#     print("#####################################")
    
#     err1, resolution1, image1 = sim.simxGetVisionSensorImage(clientID, cam1, 0, sim.simx_opmode_buffer)
#     err2, resolution2, image2 = sim.simxGetVisionSensorImage(clientID, cam2, 0, sim.simx_opmode_buffer)
#     err3, resolution3, image3 = sim.simxGetVisionSensorImage(clientID, cam3, 0, sim.simx_opmode_buffer)
#     err4, resolution4, image4 = sim.simxGetVisionSensorImage(clientID, cam4, 0, sim.simx_opmode_buffer)
    
#     ret, center_orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
#     ret1, center_position = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
#     ret2, center_position1 = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)

#     print("network connection time : ", time.time()- start)
#     ##### v-rep 카메라 설정 #####
    
#     # left 카메라
#     img1 = np.array(image1, dtype=np.uint8)
#     img1.resize([resolution1[1], resolution1[0], 3])
#     img1 = flip(img1, 0)
    
#     #right 카메라
#     img2 = np.array(image2, dtype = np.uint8)
#     img2.resize([resolution2[1], resolution2[0], 3])
#     img2 = flip(img2, 0)
    
#     # front 카메라
#     img3 = np.array(image3, dtype=np.uint8)
#     img3.resize([resolution3[1], resolution3[0], 3])
#     img3 = flip(img3, 0)
    
#     # back 카메라
#     img4 = np.array(image4, dtype=np.uint8)
#     img4.resize([resolution4[1], resolution4[0], 3])
#     img4 = flip(img4, 0)
    
    
#     ##### real time image check #####
    
#     img1_resize = cv2.resize(img1.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
#     img2_resize = cv2.resize(img2.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
#     img3_resize = cv2.resize(img3.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
#     img4_resize = cv2.resize(img4.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
    
#     # cv2.imshow('img1_original', img1_resize)
#     # cv2.imshow('img2_original', img2_resize)
#     # cv2.imshow('img3_original', img3_resize)
#     # cv2.imshow('img4_original', img4_resize)
    
#     ##### Bird view로 변환 #####
    
#     img1_bird = cv2.warpPerspective(img1.copy(), M, (476, 550))
#     img2_bird = cv2.warpPerspective(img2.copy(), M, (476, 550))
#     img3_bird = cv2.warpPerspective(img3.copy(), M, (476, 550))
#     img4_bird = cv2.warpPerspective(img4.copy(), M, (476, 550))
    
#     # cv2.imshow('img1_bird', img1_bird)
#     # cv2.imshow('img2_bird', img2_bird)
#     # cv2.imshow('img3_bird', img3_bird)
#     # cv2.imshow('img4_bird', img4_bird)

#     #####  회전 이미지 확인 #####

#     img1_bird_rotate = cv2.rotate(img1_bird.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
#     img2_bird_rotate = cv2.rotate(img2_bird.copy(), cv2.ROTATE_90_CLOCKWISE)
#     img3_bird_rotate = img3_bird.copy()
#     img4_bird_rotate = cv2.rotate(img4_bird.copy(), cv2.ROTATE_180)
    
#     # cv2.imshow('img1_bird_rotate', img1_bird_rotate)
#     # cv2.imshow('img2_bird_rotate', img2_bird_rotate)
#     # cv2.imshow('img3_bird_rotate', img3_bird_rotate)
#     # cv2.imshow('img4_bird_rotate', img4_bird_rotate)
    
    
#     ##### depth에 따른 blur 효과 제거 하기 #####
    
#     img1_bird_gray = cv2.cvtColor(img1_bird_rotate.copy(), cv2.COLOR_BGR2GRAY)
#     img2_bird_gray = cv2.cvtColor(img2_bird_rotate.copy(), cv2.COLOR_BGR2GRAY)
#     img3_bird_gray = cv2.cvtColor(img3_bird_rotate.copy(), cv2.COLOR_BGR2GRAY)
#     img4_bird_gray = cv2.cvtColor(img4_bird_rotate.copy(), cv2.COLOR_BGR2GRAY)
                                  
#     rem, cam1_white = cv2.threshold(img1_bird_gray, 100, 255, cv2.THRESH_BINARY)
#     rem, cam2_white = cv2.threshold(img2_bird_gray, 100, 255, cv2.THRESH_BINARY)
#     rem, cam3_white = cv2.threshold(img3_bird_gray, 100, 255, cv2.THRESH_BINARY)
#     rem, cam4_white = cv2.threshold(img4_bird_gray, 100, 255, cv2.THRESH_BINARY)
    
#     # cv2.imshow('cam1_white', cam1_white)
#     # cv2.imshow('cam2_white', cam2_white)
#     # cv2.imshow('cam3_white', cam3_white)
#     # cv2.imshow('cam4_white', cam4_white)

#     ##### 회전 각도 확인 #####
    
#     ##### imu 센서 로봇의 회전 각도 도출 #####
    
#     imu = round(center_orientation[2] * 180/math.pi, 2)
    
#     print("imu degree : ", imu)
    
#     ##### 라인 각도 검출을 위한 canny 적용 #####
    
#     cam1_white_canny = cv2.Canny(cam1_white.copy(), 100, 150, apertureSize = 5, L2gradient = True)
#     cam2_white_canny = cv2.Canny(cam2_white.copy(), 100, 150, apertureSize = 5, L2gradient = True)
#     cam3_white_canny = cv2.Canny(cam3_white.copy(), 100, 150, apertureSize = 5, L2gradient = True)
#     cam4_white_canny = cv2.Canny(cam4_white.copy(), 100, 150, apertureSize = 5, L2gradient = True)
    
#     # cv2.imshow('cam1_white_canny', cam1_white_canny)
#     # cv2.imshow('cam2_white_canny', cam2_white_canny)
#     # cv2.imshow('cam3_white_canny', cam3_white_canny)
#     # cv2.imshow('cam4_white_canny', cam4_white_canny)

#     ##### canny 이미지에서 각도 검출 #####

#     line1 = cv2.HoughLinesP(cam1_white_canny.copy(), 0.8, np.pi / 180, 90, minLineLength = 50, maxLineGap = 100)
#     line2 = cv2.HoughLinesP(cam2_white_canny.copy(), 0.8, np.pi / 180, 90, minLineLength = 50, maxLineGap = 100)
#     line3 = cv2.HoughLinesP(cam3_white_canny.copy(), 0.8, np.pi / 180, 90, minLineLength = 50, maxLineGap = 100)
#     line4 = cv2.HoughLinesP(cam4_white_canny.copy(), 0.8, np.pi / 180, 90, minLineLength = 50, maxLineGap = 100)
    
#     ##### 도출된 line이 없는 경우에 대한 대책 #####
    
#     if line1 is None:
#         line1 = []
#     elif line2 is None:
#         line2 = []
#     elif line3 is None:
#         line3 = []
#     elif line4 is None:
#         line4 = []
    
#     ##### line이 가장 많은 사진을 선택 #####
    
#     line_list = [len(line1), len(line2), len(line3), len(line4)]
#     list_number = np.argmax(line_list)
    
#     if list_number == 0:
#         many_line = line1
#         select_camera = "left camera"
#     elif list_number == 1:
#         many_line = line2
#         select_camera = "right camera"
#     elif list_number == 2:
#         many_line = line3
#         select_camera = "front camera"
#     elif list_number == 3:
#         many_line = line4
#         select_camera = "back camera"
    
#     print("many line camera : ", select_camera)

#     #print("line list",line_list)
    
#     ##### 선택된 사진을 통해서 이미지 각도 추출 #####
    
#     degree = []
#     image_degree = []
    
#     for i in many_line:
        
#         image = np.zeros((476, 550,3), np.uint8)
#         cv2.line(image,(i[0][0], i[0][1]), (i[0][2], i[0][3]), (0, 0, 255), 2)
#         cv2.line(image, (238, 0), (238, 550), (0, 0, 255), 2)    
# #         cv2.imshow("image", image)

#         image = np.mean(image, axis=2)
#         hspace, angles, distances = hough_line(image)

#         angle=[]
                
#         for _, a , distances in zip(*hough_line_peaks(hspace, angles, distances)):
#             angle.append(a)

#         angles = [a*180/np.pi for a in angle]

#         angle_difference = np.max(angles) - np.min(angles)
        
#         degree.append(angle_difference)
#         print(angle_difference)
#         # cv2.imshow("image", image)
    
#     ##### imu 센서 도출 각도와 이미지 각도 비교 #####
    
#     if 0 <= imu and imu < 90:
#         new_imu = imu
#         for p in range(len(degree)):
#             if new_imu - 5 <= degree[p] and new_imu + 5 >= degree[p]:
#                 image_degree.append(degree[p])
        
#     elif -90 <= imu and imu < 0:
#         new_imu = - imu
#         for p in range(len(degree)):
#             if new_imu - 5 <= degree[p] and new_imu + 5 >= degree[p]:
#                 image_degree.append(-degree[p])
        
#     elif 90 <= imu and imu < 180:
#         new_imu = imu
#         for p in range(len(degree)):
#             if new_imu - 5 <= 180 - degree[p]  and new_imu + 5 >= 180 - degree[p] :        
#                 image_degree.append(180 - degree[p] )
        
        
#     elif -180 < imu and imu < -90:
#         new_imu = -imu
#         for p in range(len(degree)):
#             if new_imu - 5 <= 180 - degree[p]  and new_imu + 5 >= 180 - degree[p] :    
#                 image_degree.append(degree[p] - 180 )
                
#     elif imu == -180 or imu == 180:
#         new_imu = abs(imu)
#         for p in range(len(degree)):
#             if new_imu - 5 <= degree[p] + 90 and new_imu + 5 >= degree[p] + 90:
#                 image_degree.append(degree[p] + 90)
    
#     # print(image_degree)

#     if image_degree == []:
#         continue
#     else:
#         mean_degree = np.mean(image_degree)
#         if mean_degree <-180:
#             mean_degree += 360
    
#         if mean_degree > 180:
#             mean_degree -= 360
        
#         list_angle.append(mean_degree)
    
#     print("image rotation degree : " ,mean_degree)

#     text_degree = "rotation degree : "+str(round(mean_degree,3))+" deg"

#     diff_degree = (degree_list[-1] - mean_degree)
    
#     ##### 이미지 회전 #####
    
#     ##### image padding #####
    
#     black = [0, 0, 0]
    
#     # 126 126 88 88
    
#     cam1_white_pad = cv2.copyMakeBorder(cam1_white, 127, 127, 89, 89, cv2.BORDER_CONSTANT, value = black)
#     cam2_white_pad = cv2.copyMakeBorder(cam2_white, 127, 127, 89, 89, cv2.BORDER_CONSTANT, value = black)
#     cam3_white_pad = cv2.copyMakeBorder(cam3_white, 89, 89, 127, 127, cv2.BORDER_CONSTANT, value = black)
#     cam4_white_pad = cv2.copyMakeBorder(cam4_white, 89, 89, 127, 127, cv2.BORDER_CONSTANT, value = black)
    
#     # cv2.imshow("cam1_white_pad", cam1_white_pad)
#     # cv2.imshow("cam2_white_pad", cam2_white_pad)
#     # cv2.imshow("cam3_white_pad", cam3_white_pad)
#     # cv2.imshow("cam4_white_pad", cam4_white_pad)
    
#     cam1_white_rotate = rotate(cam1_white_pad, mean_degree, 1)
#     cam2_white_rotate = rotate(cam2_white_pad, mean_degree, 1)
#     cam3_white_rotate = rotate(cam3_white_pad, mean_degree, 1)
#     cam4_white_rotate = rotate(cam4_white_pad, mean_degree, 1)      

#     # cam1_white_rotate_scale = cv2.resize(cam1_white_rotate.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
#     # cam2_white_rotate_scale = cv2.resize(cam2_white_rotate.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
#     # cam3_white_rotate_scale = cv2.resize(cam3_white_rotate.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
#     # cam4_white_rotate_scale = cv2.resize(cam4_white_rotate.copy(), None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)
    
#     # cv2.imshow("cam1_white_rotate", cam1_white_rotate_scale)
#     # cv2.imshow("cam2_white_rotate", cam2_white_rotate_scale)
#     # cv2.imshow("cam3_white_rotate", cam3_white_rotate_scale)
#     # cv2.imshow("cam4_white_rotate", cam4_white_rotate_scale)
    
#     ##### 이미지에 대한 침식, 팽창 연산 #####
    
#     kernel1 = np.ones((1, 30), np.uint8)
#     kernel2 = np.ones((30, 1), np.uint8)
#     kernel3 = np.ones((2,2), np.uint8)

#     cam1_closing = cv2.morphologyEx(cam1_white_rotate, cv2.MORPH_CLOSE, kernel1)
#     cam1_closing = cv2.morphologyEx(cam1_closing, cv2.MORPH_CLOSE, kernel2)
    
#     cam2_closing = cv2.morphologyEx(cam2_white_rotate, cv2.MORPH_CLOSE, kernel1)
#     cam2_closing = cv2.morphologyEx(cam2_closing, cv2.MORPH_CLOSE, kernel2)

#     cam3_closing = cv2.morphologyEx(cam3_white_rotate, cv2.MORPH_CLOSE, kernel1)
#     cam3_closing = cv2.morphologyEx(cam3_closing, cv2.MORPH_CLOSE, kernel2)
    
#     cam4_closing = cv2.morphologyEx(cam4_white_rotate, cv2.MORPH_CLOSE, kernel1)
#     cam4_closing = cv2.morphologyEx(cam4_closing, cv2.MORPH_CLOSE, kernel2)
    
#     # cv2.imshow("cam1_closing", cam1_closing)
#     # cv2.imshow("cam2_closing", cam2_closing)
#     # cv2.imshow("cam3_closing", cam3_closing)
#     # cv2.imshow("cam4_closing", cam4_closing)
    
#     cam1_closing_undo = rotate(cam1_closing, -mean_degree, 1)
#     cam2_closing_undo = rotate(cam2_closing, -mean_degree, 1)
#     cam3_closing_undo = rotate(cam3_closing, -mean_degree, 1)
#     cam4_closing_undo = rotate(cam4_closing, -mean_degree, 1)  
    
#     # cv2.imshow("cam1_closing", cam1_closing_undo)
#     # cv2.imshow("cam2_closing", cam2_closing_undo)
#     # cv2.imshow("cam3_closing", cam3_closing_undo)
#     # cv2.imshow("cam4_closing", cam4_closing_undo)

#     cam1_closing_undo = cam1_closing_undo[89:639,89:639]
#     cam2_closing_undo = cam2_closing_undo[89:639,89:639]
#     cam3_closing_undo = cam3_closing_undo[89:639,89:639]
#     cam4_closing_undo = cam4_closing_undo[89:639,89:639]
    
#     # cv2.imshow("cam1_closing_undo", cam1_closing_undo)
#     # cv2.imshow("cam2_closing_undo", cam2_closing_undo)
#     # cv2.imshow("cam3_closing_undo", cam3_closing_undo)
#     # cv2.imshow("cam4_closing_undo", cam4_closing_undo)
    
#     ##### 단일 스케일 매칭 #####
    
#     ##### scale 불러오기 #####
    
#     pin = open('new_scale.txt', 'r')
    
#     scale_num = pin.readline()
    
#     scale_num = float(scale_num)
    
#     #print("scale ratio : ",scale_num)
    
#     pin.close()
    
#     ##### template 이미지 정의 #####
    
#     template1 = cam1_closing_undo  #grayscale
#     template2 = cam2_closing_undo  #grayscale
#     template3 = cam3_closing_undo  #grayscale
#     template4 = cam4_closing_undo  #grayscale
    
#     template1_resize = cv2.resize(template1.copy(), None, fx = 0.85, fy = 0.85, interpolation = cv2.INTER_AREA)
#     template2_resize = cv2.resize(template2.copy(), None, fx = 0.85, fy = 0.85, interpolation = cv2.INTER_AREA)
#     template3_resize = cv2.resize(template3.copy(), None, fx = 0.85, fy = 0.85, interpolation = cv2.INTER_AREA)
#     template4_resize = cv2.resize(template4.copy(), None, fx = 0.85, fy = 0.85, interpolation = cv2.INTER_AREA)

#     text1 = "Left"
#     text2 = "Right"
#     text3 = "Front"
#     text4 = "Back"

#     cv2.putText(template1_resize, text1, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255,255,255), 2, cv2.LINE_AA)
#     cv2.putText(template2_resize, text2, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255,255,255), 2, cv2.LINE_AA)
#     cv2.putText(template3_resize, text3, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255,255,255), 2, cv2.LINE_AA)
#     cv2.putText(template4_resize, text4, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255,255,255), 2, cv2.LINE_AA)


#     template_all = cv2.hconcat([template1_resize, template2_resize, template3_resize, template4_resize])
    
#     cv2.imshow("template image", template_all)
#     #cv2.imshow("template1_resize", template1_resize)
#     # cv2.imshow("template2_resize", template2_resize)
#     # cv2.imshow("template3_resize", template3_resize)
#     # cv2.imshow("template4_resize", template4_resize)
    
#     (template1_height, template1_width) = template1.shape[:2]
#     (template2_height, template2_width) = template2.shape[:2]
#     (template3_height, template3_width) = template3.shape[:2]
#     (template4_height, template4_width) = template4.shape[:2]
    
#     ##### blue print에 scale 적용 #####

#     blue_print = cv2.imread('image/blue_print/court_test_change.png', cv2.IMREAD_COLOR)
#     blue_print_rot = rotate(blue_print.copy(), -mean_degree, 0)
#     blue_print_gray = cv2.cvtColor(blue_print_rot.copy(), cv2.COLOR_BGR2GRAY)

#     blue_print_reszie = cv2.resize(blue_print_rot.copy(), None, fx = 0.25, fy = 0.25, interpolation = cv2.INTER_AREA)
    
#     #cv2.imshow('blue_print_gray', blue_print_reszie)
    
#     blue_print_scale = imutils.resize(blue_print_gray.copy(), width = int(blue_print_gray.shape[1] * 0.4630630630630631))
    
#     blue_print_canny = cv2.Canny(blue_print_scale, 100, 150)
    
#     # 0.4332665330661323
#     # 0.4630630630630631
    
#     blue_print1 = blue_print_rot.copy()
#     blue_print2 = blue_print_rot.copy()
#     blue_print3 = blue_print_rot.copy()
#     blue_print4 = blue_print_rot.copy()
#     QWERT = blue_print_rot.copy()  
#     final = blue_print_rot.copy()
#     black_path = blue_print_rot.copy()
#     position_map = blue_print_rot.copy()
    
#     ##### scale 비율에 따른 pixel 위치 계산 변수 #####
    
#     r = blue_print_gray.shape[1] / float(blue_print_scale.shape[1])
    
#     ##### 단일 scale template_matching #####
    
#     # maxLoc 이용 : cv.TM_CCOEFF / cv.TM_CCOEFF_NORMED / cv.TM_CCORR / cv.TM_CCORR_NORMED / 
    
#     # minLoc 이용 : cv.TM_SQDIFF / cv.TM_SQDIFF_NORMED
    
#     ##########################################################################
#     #### gpu ####

#     gsrc = cv2.cuda_GpuMat()

#     gtmpl1 = cv2.cuda_GpuMat()
#     gtmpl2 = cv2.cuda_GpuMat()
#     gtmpl3 = cv2.cuda_GpuMat()
#     gtmpl4 = cv2.cuda_GpuMat()

#     gresult1 = cv2.cuda_GpuMat()
#     gresult2 = cv2.cuda_GpuMat()
#     gresult3 = cv2.cuda_GpuMat()
#     gresult4 = cv2.cuda_GpuMat()

#     gsrc.upload(blue_print_canny)
#     gtmpl1.upload(template1)
#     gtmpl2.upload(template2)
#     gtmpl3.upload(template3)
#     gtmpl4.upload(template4)

#     matcher = cv2.cuda.createTemplateMatching(cv2.CV_8UC1, cv2.TM_CCOEFF)

#     gresult1 = matcher.match(gsrc, gtmpl1)
#     gresult2 = matcher.match(gsrc, gtmpl2)
#     gresult3 = matcher.match(gsrc, gtmpl3)
#     gresult4 = matcher.match(gsrc, gtmpl4)

#     result1 = gresult1.download()
#     result2 = gresult2.download()
#     result3 = gresult3.download()
#     result4 = gresult4.download()

#     ####################################################################################

#     (_, maxVal1, minLoc1, maxLoc1) = cv2.minMaxLoc(result1)
#     (_, maxVal2, minLoc2, maxLoc2) = cv2.minMaxLoc(result2)
#     (_, maxVal3, minLoc3, maxLoc3) = cv2.minMaxLoc(result3)
#     (_, maxVal4, minLoc4, maxLoc4) = cv2.minMaxLoc(result4)
    
#     # print("maxVal 1 : ", maxVal1)
#     # print("maxVal 2 : ", maxVal2)
#     # print("maxVal 3 : ", maxVal3)
#     # print("maxVal 4 : ", maxVal4)

#     ##### 매칭된 사각형의 중심, 시작점 및 끝점 계산 #####
    
#     (startX1, startY1) = (int(maxLoc1[0] * r), int(maxLoc1[1] * r))
#     (endX1, endY1) = (int((maxLoc1[0] + template1_width) * r), int((maxLoc1[1] + template1_height) * r))
#     (centerX1, centerY1) = (int(startX1 + ((endX1 - startX1)/2)), int(startY1 + ((endY1 - startY1)/2)))
    
#     (startX2, startY2) = (int(maxLoc2[0] * r), int(maxLoc2[1] * r))
#     (endX2, endY2) = (int((maxLoc2[0] + template2_width) * r), int((maxLoc2[1] + template2_height) * r))
#     (centerX2, centerY2) = (int(startX2 + ((endX2 - startX2)/2)), int(startY2 + ((endY2 - startY2)/2)))
    
#     (startX3, startY3) = (int(maxLoc3[0] * r), int(maxLoc3[1] * r))
#     (endX3, endY3) = (int((maxLoc3[0] + template3_width) * r), int((maxLoc3[1] + template3_height) * r))
#     (centerX3, centerY3) = (int(startX3 + ((endX3 - startX3)/2)), int(startY3 + ((endY3 - startY3)/2)))
    
#     (startX4, startY4) = (int(maxLoc4[0] * r), int(maxLoc4[1] * r))
#     (endX4, endY4) = (int((maxLoc4[0] + template4_width) * r), int((maxLoc4[1] + template4_height) * r))
#     (centerX4, centerY4) = (int(startX4 + ((endX4 - startX4)/2)), int(startY4 + ((endY4 - startY4)/2)))
    
#     # print("left camera bounding box center position X axis : ", centerX1," Y axis : ", centerY1)
#     # print("right camera bounding box center position X axis : ", centerX2," Y axis : ", centerY2)
#     # print("front camera bounding box center position X axis : ", centerX3," Y axis : ", centerY3)
#     # print("back camera bounding box center position X axis : ", centerX4," Y axis : ", centerY4)

#     ##################################################################

#     com_rotate_robot = result_point(robot_x[-1], robot_y[-1], height1, width1, -diff_degree)

#     com_rotate_robot_x_float = com_rotate_robot[0][0]
#     com_rotate_robot_y_float = com_rotate_robot[1][0]

#     com_rotate_robot_x = int(com_rotate_robot_x_float)
#     com_rotate_robot_y = int(com_rotate_robot_y_float)

#     com_rotate_robot_x1 = com_rotate_robot_x - 727
#     com_rotate_robot_y1 = com_rotate_robot_y

#     com_rotate_robot_x2 = com_rotate_robot_x + 727
#     com_rotate_robot_y2 = com_rotate_robot_y

#     com_rotate_robot_x3 = com_rotate_robot_x 
#     com_rotate_robot_y3 = com_rotate_robot_y - 727

#     com_rotate_robot_x4 = com_rotate_robot_x 
#     com_rotate_robot_y4 = com_rotate_robot_y + 727

#     initial_x1.append(com_rotate_robot_x1)
#     initial_y1.append(com_rotate_robot_y1)

#     initial_x2.append(com_rotate_robot_x2)
#     initial_y2.append(com_rotate_robot_y2)

#     initial_x3.append(com_rotate_robot_x3)
#     initial_y3.append(com_rotate_robot_y3)

#     initial_x4.append(com_rotate_robot_x4)
#     initial_y4.append(com_rotate_robot_y4)
   

#     # degree_initial_1 = result_point(initial_x1[-1], initial_y1[-1], height1, width1, -diff_degree)
#     # degree_initial_2 = result_point(initial_x2[-1], initial_y2[-1], height1, width1, -diff_degree)
#     # degree_initial_3 = result_point(initial_x3[-1], initial_y3[-1], height1, width1, -diff_degree)
#     # degree_initial_4 = result_point(initial_x4[-1], initial_y4[-1], height1, width1, -diff_degree)

#     # degree_initial_x1_float = degree_initial_1[0][0]
#     # degree_initial_y1_float = degree_initial_1[1][0]

#     # degree_initial_x2_float = degree_initial_2[0][0]
#     # degree_initial_y2_float = degree_initial_2[1][0]

#     # degree_initial_x3_float = degree_initial_3[0][0]
#     # degree_initial_y3_float = degree_initial_3[1][0]

#     # degree_initial_x4_float = degree_initial_4[0][0]
#     # degree_initial_y4_float = degree_initial_4[1][0]

#     # degree_initial_x1 = int(degree_initial_x1_float)
#     # degree_initial_y1 = int(degree_initial_y1_float)

#     # degree_initial_x2 = int(degree_initial_x2_float)
#     # degree_initial_y2 = int(degree_initial_y2_float)

#     # degree_initial_x3 = int(degree_initial_x3_float)
#     # degree_initial_y3 = int(degree_initial_y3_float)

#     # degree_initial_x4 = int(degree_initial_x4_float)
#     # degree_initial_y4 = int(degree_initial_y4_float)

#     # initial_x1.append(degree_initial_x1)
#     # initial_y1.append(degree_initial_y1)

#     # initial_x1.append(degree_initial_x2)
#     # initial_y1.append(degree_initial_y2)

#     # initial_x1.append(degree_initial_x3)
#     # initial_y1.append(degree_initial_y3)

#     # initial_x1.append(degree_initial_x4)
#     # initial_y1.append(degree_initial_y4)

#     ##################################################################
    

#     if abs(abs(centerX1) - abs(initial_x1[-1])) <= 150 and abs(abs(centerY1) - abs(initial_y1[-1])) <= 150:
#         initial_x1.append(centerX1)
#         initial_y1.append(centerY1)
#     else :
#         maxVal1 /= 10000
    
#     if abs(abs(centerX2) - abs(initial_x2[-1])) <= 150 and abs(abs(centerY2) - abs(initial_y2[-1])) <= 150:
#         initial_x2.append(centerX2)
#         initial_y2.append(centerY2)
#     else :
#         maxVal2 /= 10000

#     if abs(abs(centerX3) - abs(initial_x3[-1])) <= 150 and abs(abs(centerY3) - abs(initial_y3[-1])) <= 150:
#         initial_x3.append(centerX3)
#         initial_y3.append(centerY3)
#     else :
#         maxVal3 /= 10000

#     if abs(abs(centerX4) - abs(initial_x4[-1])) <= 150 and abs(abs(centerY4) - abs(initial_y4[-1])) <= 150:
#         initial_x4.append(centerX4)
#         initial_y4.append(centerY4)
#     else :
#         maxVal4 /= 10000

#     # print("maxVal 1 :", maxVal1 )
#     # print("maxVal 2 :", maxVal2 )
#     # print("maxVal 3 :", maxVal3 )
#     # print("maxVal 4 :", maxVal4 )

#     # [4]
#     if maxVal1 >= 10000000 and maxVal2 >= 10000000 and maxVal3 >= 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = ((centerX1+centerX2+centerX3+centerX4) / 4, (centerY1 + centerY2 + centerY3 + centerY4) / 4) 
#         print("all matching!!!")
#         text_matching = "all matching"
#     # [3-1]
#     elif maxVal1 >= 10000000 and maxVal2 >= 10000000 and maxVal3 >= 10000000 and maxVal4 < 10000000:
#         (robot_x3, robot_y3) = ((centerX1+centerX2+centerX3) / 3, (centerY1 + centerY2) / 2) 
#         print("left, right, front matching!!")
#         text_matching = "left, right, front matching"
#     # [3-2]
#     elif maxVal1 >= 10000000 and maxVal2 >= 10000000 and maxVal3 < 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = ((centerX1+centerX2+centerX4) / 3, (centerY1 + centerY2) / 2) 
#         print("left, rihgt, back matching!!")
#         text_matching = "left, rihgt, back matching"
#     # [3-3]
#     elif maxVal1 >= 10000000 and maxVal2 < 10000000 and maxVal3 >= 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = ((centerX3+centerX4) / 2, (centerY1 + centerY3 + centerY4) / 3) 
#         print("left, front, back matching!!")
#         text_matching = "left, front, back matching"
#     # [3-4]
#     elif maxVal1 < 10000000 and maxVal2 >= 10000000 and maxVal3 >= 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = ((centerX3+centerX4) / 2, (centerY2 + centerY3 + centerY4) / 3) 
#         print("right, front, back matching!!")
#         text_matching = "right, front, back matching"
#     # [2-1]
#     elif maxVal1 >= 10000000 and maxVal2 >= 10000000 and maxVal3 < 10000000 and maxVal4 < 10000000:
#         (robot_x3, robot_y3) = ((centerX1+centerX2) / 2, (centerY1 + centerY2) / 2) 
#         print("left, right matching!!")
#         text_matching = "left, right matching"
#     # [2-2]
#     elif maxVal1 >= 10000000 and maxVal2 < 10000000 and maxVal3 >= 10000000 and maxVal4 < 10000000:
#         (robot_x3, robot_y3) = (centerX3, centerY1) 
#         print("left, front matching!!")
#         text_matching = "left, front matching"
#     # [2-3]
#     elif maxVal1 >= 10000000 and maxVal2 < 10000000 and maxVal3 < 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = (centerX4, centerY1)    
#         print("left, back matching!!")
#         text_matching = "left, back matching"
#     # [2-4]
#     elif maxVal1 < 10000000 and maxVal2 >= 10000000 and maxVal3 >= 10000000 and maxVal4 < 10000000:
#         (robot_x3, robot_y3) = (centerX3, centerY2)
#         print("right, front matching!!")
#         text_matching = "right, front matching"
#     # [2-5]
#     elif maxVal1 < 10000000 and maxVal2 >= 10000000 and maxVal3 < 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = (centerX4, centerY2)
#         print("right, back matching")
#         text_matching = "right, back matching"
#     # [2-6]
#     elif maxVal1 < 10000000 and maxVal2 < 10000000 and maxVal3 >= 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = ((centerX3+centerX4) / 2, (centerY3 + centerY4) / 2) 
#         print("front, back matching")
#         text_matching = "front, back matching"
#     # [1-1]
#     elif maxVal1 >= 10000000 and maxVal2 < 10000000 and maxVal3 < 10000000 and maxVal4 < 10000000:
#         (robot_x3, robot_y3) = roi_point_left2right(centerX1, centerY1, 1454, mean_degree)
#         print("left matching only!!")
#         text_matching = "left matching only"
#     # [1-2]
#     elif maxVal1 < 10000000 and maxVal2 >= 10000000 and maxVal3 < 10000000 and maxVal4 < 10000000:
#         (robot_x3, robot_y3) = roi_point_right2left(centerX2, centerY2, 1454, mean_degree)
#         print("right matching only!!")
#         text_matching = "right matching only"
#     # [1-3]
#     elif maxVal1 < 10000000 and maxVal2 < 10000000 and maxVal3 >= 10000000 and maxVal4 < 10000000:
#         (robot_x3, robot_y3) = roi_point_front2back(centerX3, centerY3, 1454, mean_degree)
#         print("front matching only!!")
#         text_matching = "front matching only"
#     # [1-4]
#     elif maxVal1 < 10000000 and maxVal2 < 10000000 and maxVal3 < 10000000 and maxVal4 >= 10000000:
#         (robot_x3, robot_y3) = roi_point_back2front(centerX4, centerY4, 1454, mean_degree)
#         print("back matching only!!")
#         text_matching = "back matching only"

#     elif maxVal1 < 10000000 and maxVal2 < 10000000 and maxVal3 < 10000000 and maxVal4 < 10000000:
#         Val = [maxVal1, maxVal2, maxVal3, maxVal4]
#         if maxVal1 == max(Val):
#             (robot_x3, robot_y3) = (centerX1 + 727, centerY1)
#             print("111111")
#             text_matching = "singularity : left matching only"
#         elif maxVal2 == max(Val):
#             (robot_x3, robot_y3) = (centerX2 - 727, centerY2)
#             print("222222")
#             text_matching = "singularity : right matching only"
#         elif maxVal3 == max(Val):
#             (robot_x3, robot_y3) = (centerX3, centerY3 + 727)
#             print("333333")
#             text_matching = "singularity : front matching only"
#         elif maxVal4 == max(Val):
#             (robot_x3, robot_y3) = (centerX4, centerY4 - 727)
#             print("444444")
#             text_matching = "singularity : back matching only"
#     else:
#         continue

#     ###############################################################################################################

#     rotate_robot = result_point(robot_x3, robot_y3, height1, width1, diff_degree)
#     rotate_robot_int = rotate_robot.astype(np.int64) 

#     rotate_robot_pos_x_float = rotate_robot[0][0]
#     rotate_robot_pos_y_float = rotate_robot[1][0]

#     rotate_robot_pos_x = int(rotate_robot_pos_x_float)
#     rotate_robot_pos_y = int(rotate_robot_pos_y_float)

#     robot_x.append(rotate_robot_pos_x)
#     robot_y.append(rotate_robot_pos_y)

#     rotate_center_x1 = rotate_robot_pos_x - 727
#     rotate_center_y1 = rotate_robot_pos_y

#     rotate_center_x2 = rotate_robot_pos_x + 727
#     rotate_center_y2 = rotate_robot_pos_y

#     rotate_center_x3 = rotate_robot_pos_x 
#     rotate_center_y3 = rotate_robot_pos_y - 727

#     rotate_center_x4 = rotate_robot_pos_x 
#     rotate_center_y4 = rotate_robot_pos_y + 727

#     initial_x1.append(rotate_center_x1)
#     initial_y1.append(rotate_center_y1)

#     initial_x2.append(rotate_center_x2)
#     initial_y2.append(rotate_center_y2)

#     initial_x3.append(rotate_center_x3)
#     initial_y3.append(rotate_center_y3)

#     initial_x4.append(rotate_center_x4)
#     initial_y4.append(rotate_center_y4)


#     ##### blue print에 매칭된 위치 표시 #####
    
#     red = (0, 0, 255)
    
#     cv2.circle(blue_print1, (centerX1, centerY1), 5, (0, 255, 0), 15)
#     cv2.rectangle(blue_print1, (startX1, startY1), (endX1, endY1), red, 20)
    
#     cv2.circle(blue_print2, (centerX2, centerY2), 5, (0, 255, 0), 15)
#     cv2.rectangle(blue_print2, (startX2, startY2), (endX2, endY2), red, 20)

#     cv2.circle(blue_print3, (centerX3, centerY3), 5, (0, 255, 0), 15)
#     cv2.rectangle(blue_print3, (startX3, startY3), (endX3, endY3), red, 20)

#     cv2.circle(blue_print4, (centerX4, centerY4), 5, (0, 255, 0), 15)
#     cv2.rectangle(blue_print4, (startX4, startY4), (endX4, endY4), red, 20)
    
#     cv2.rectangle(final, (startX1, startY1), (endX1, endY1), (255, 255, 0), 20)
#     cv2.rectangle(final, (startX2, startY2), (endX2, endY2), (255, 255, 0), 20)
#     cv2.rectangle(final, (startX3, startY3), (endX3, endY3), (255, 255, 0), 20)
#     cv2.rectangle(final, (startX4, startY4), (endX4, endY4), (255, 255, 0), 20)

#     cv2.circle(final, (centerX1, centerY1), 10, (200, 200, 200), 10)
#     cv2.circle(final, (com_rotate_robot_x1, com_rotate_robot_y1), 10, (100, 100, 100), 10)
#     cv2.circle(final, (com_rotate_robot_x2, com_rotate_robot_y2), 10, (0,255,0), 10)
#     cv2.circle(final, (com_rotate_robot_x3, com_rotate_robot_y3), 10, (0,255,0), 10)
#     cv2.circle(final, (com_rotate_robot_x4, com_rotate_robot_y4), 10, (0,255,0), 10)

#     cv2.circle(final, (robot_x3, robot_y3), 10, (0,0,255), 10)
#     cv2.circle(roro_image, (rotate_robot_pos_x, rotate_robot_pos_y), 10, (0,0,255), 10)

#     final_image = cv2.resize(final, None, fx = 0.18, fy = 0.18, interpolation = cv2.INTER_AREA)
#     roro = cv2.resize(roro_image, None, fx = 0.25, fy = 0.25, interpolation = cv2.INTER_AREA)

#     cv2.putText(final_image, text_matching, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255,255,255), 2, cv2.LINE_AA)
#     cv2.putText(final_image, text_degree, (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255,255,255), 2, cv2.LINE_AA)

#     cv2.imshow("final_image",final_image)
#     #cv2.imshow("roro", roro)
#     # # ########################################

#     xxx = rotate_robot_pos_x
#     yyy = rotate_robot_pos_y
    
#     measure_robot_x.append(xxx)
#     measure_robot_y.append(yyy)

#     #################################
#     #매칭 알고리즘만 사용한 것

#     ##################################
    
#     draw_center_x_k = center_position1[0] + 5.485
#     draw_center_y_k = center_position1[1] - 0.0245

#     draw_center_x1_k = int(abs(draw_center_x_k * 1000 / 4.810943396 )) + 1486 + 7
#     draw_center_y1_k = int(abs(draw_center_y_k * 1000 / 4.810943396 )) + 1335 - 2

#     real_x_k.append(draw_center_x1_k)
#     real_y_k.append(draw_center_y1_k)

#     draw_center_x = center_position[0] + 5.485
#     draw_center_y = center_position[1] - 0.0245

#     draw_center_x1 = int(abs(draw_center_x * 1000 / 4.810943396 )) + 1486 + 7
#     draw_center_y1 = int(abs(draw_center_y * 1000 / 4.810943396 )) + 1335 - 2
    
#     real_x.append(draw_center_x1)
#     real_y.append(draw_center_y1)

#     estimage_path_image2_k = cv2.circle(real_moving_image_k, (draw_center_x1_k, draw_center_y1_k), 10, (0, 100, 100), 10)
#     print(abs(abs(kalman_measure_x_k[-1])-abs(xxx)))
#     print(abs(abs(kalman_measure_y_k[-1])-abs(yyy)))

#     #kalman_filter
#     if abs(abs(measure_robot_x[-2])-abs(xxx)) <= 140 and abs(abs(measure_robot_y[-2])-abs(yyy)) <= 140 and t == 0:
#         kalman_measure_x_k.append(xxx)
#         kalman_measure_y_k.append(yyy)

#         xxxx = kalman_measure_x_k[-1]
#         yyyy = kalman_measure_y_k[-1]

#         t+=1

#     if abs(abs(kalman_measure_x_k[-1])-abs(xxx)) <= 140 and abs(abs(kalman_measure_y_k[-1])-abs(yyy)) <= 140 and t >= 1:
        
#         kalman_measure_x_k.append(xxx)
#         kalman_measure_y_k.append(yyy)

#         xxxx = kalman_measure_x_k[-1]
#         yyyy = kalman_measure_y_k[-1]
#         print('a')


#     array_3 = np.array([xxxx])
#     array_4 = np.array([yyyy])
#     z_meas_k= np.append(array_3, array_4)
    
#     if kalman_k == 0:
#         x_esti_k, P_k = x_0, P_0

#         xpos_meas_save_k = np.array([z_meas_k[0]])
#         ypos_meas_save_k = np.array([z_meas_k[1]])
#         xpos_esti_save_k = np.array([x_esti_k[0]])
#         ypos_esti_save_k = np.array([x_esti_k[2]])

#         x_pos_esti_k = int(xpos_esti_save_k)
#         y_pos_esti_k = int(ypos_esti_save_k)

#         kalman_k +=1

#     else:
#         x_esti_k, P_k = kalman_filter_k(z_meas_k, x_esti_k, P_k)
    
#         xpos_meas_save_k = np.array([z_meas_k[0]])
#         ypos_meas_save_k = np.array([z_meas_k[1]])
#         xpos_esti_save_k = np.array([x_esti_k[0]])
#         ypos_esti_save_k = np.array([x_esti_k[2]])

#         x_pos_esti_k = int(xpos_esti_save_k)
#         y_pos_esti_k = int(ypos_esti_save_k)

#         kalman_k +=1

#     kalman_x_k.append(x_pos_esti_k)
#     kalman_y_k.append(y_pos_esti_k)

#     estimage_path_image3_k = cv2.circle(estimage_path_image2_k, (x_pos_esti_k, y_pos_esti_k), 10, (255,0,0), 10)

#     real_kalman_image_k = cv2.resize(estimage_path_image3_k, None, fx = 0.18, fy = 0.18, interpolation = cv2.INTER_AREA)

#     text5 = "Kalman Filter"
    
#     cv2.putText(real_kalman_image_k, text5, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 2, cv2.LINE_AA)

#     #cv2.imshow("real_kalman_image_k", real_kalman_image_k)

#     esti_XX_k = int(x_pos_esti_k)
#     esti_YY_k = int(y_pos_esti_k)

#     esti_XXX_k = round((esti_XX_k - 1486 + 7) * 4.810943396 / 1000, 3)
#     esti_YYY_k = round((esti_YY_k - 1335 - 2) * 4.810943396 / 1000, 3)

#     center_position1[0] += 5.485
#     center_position1[1] -= 0.0245

#     error_esti_xx_k = abs(abs(center_position1[0]) - esti_XXX_k)
#     error_esti_yy_k = abs(abs(center_position1[1]) - esti_YYY_k)

#     text_esti_XXX_k = "X axis : "+str(esti_XXX_k)+"m"
#     text_esti_YYY_k = "Y axis : "+str(esti_YYY_k)+"m"
#     text_X = "x"
#     text_Y = "y"

#     error_kalman_x_k.append(error_esti_xx_k)
#     error_kalman_y_k.append(error_esti_yy_k)

#     cv2.putText(real_kalman_image_k, text_esti_XXX_k, (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 2, cv2.LINE_AA)
#     cv2.putText(real_kalman_image_k, text_esti_YYY_k, (10,120), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 2, cv2.LINE_AA)
#     cv2.putText(real_kalman_image_k, text_X, (348,232), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,255), 2, cv2.LINE_AA)
#     cv2.putText(real_kalman_image_k, text_Y, (238,322), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,255,0), 2, cv2.LINE_AA)
#     cv2.arrowedLine(real_kalman_image_k, (268,242), (348,242), (0,0,255), 4)
#     cv2.arrowedLine(real_kalman_image_k, (268,242), (268,322), (0,255,0), 4)

#     print("________________kalman filter x error___________________",error_esti_xx_k)
#     print("________________kalman filter y error___________________",error_esti_yy_k)

#     cv2.imshow("real_kalman_image_k", real_kalman_image_k)
#     # array_1 = np.array([xxx])
#     # array_2 = np.array([yyy])
#     # z_meas = np.append(array_1, array_2)

#     # if kalman_u == 0:
#     #     x_esti, P = x_0, P_0

#     #     xpos_meas_save = np.array([z_meas[0]])
#     #     ypos_meas_save = np.array([z_meas[1]])
#     #     xpos_esti_save = np.array([x_esti[0]])
#     #     ypos_esti_save = np.array([x_esti[2]])

#     #     x_pos_esti = int(xpos_esti_save)
#     #     y_pos_esti = int(ypos_esti_save)

#     #     kalman_u +=1


#     # else:
#     #     x_esti, P = kalman_filter(z_meas, x_esti, P)
        
#     #     xpos_meas_save = np.array([z_meas[0]])
#     #     ypos_meas_save = np.array([z_meas[1]])
#     #     xpos_esti_save = np.array([x_esti[0]])
#     #     ypos_esti_save = np.array([x_esti[2]])

#     #     x_pos_esti = int(xpos_esti_save)
#     #     y_pos_esti = int(ypos_esti_save)

#     #     kalman_u +=1

#     # kalman_x.append(x_pos_esti)
#     # kalman_y.append(y_pos_esti)

#     # # 이미지 매칭 알고리즘 좌표 + 실제 좌표
#     kalman_image_measure_robot = cv2.circle(kalman_image, (draw_center_x1, draw_center_y1), 10, (100, 100, 100), 10)
#     estimage_path_image0 = cv2.circle(kalman_image_measure_robot, (xxx, yyy), 10, (0, 0, 255), 10)

#     # # 실제 좌표 + 칼만필터
#     # estimage_path_image2 = cv2.circle(real_moving_image, (draw_center_x1, draw_center_y1), 10, (0, 102, 102), 10)
#     # estimage_path_image3 = cv2.circle(real_moving_image, (x_pos_esti, y_pos_esti), 10, (255,0,0), 10)

#     real_measure_image = cv2.resize(estimage_path_image0, None, fx = 0.18, fy = 0.18, interpolation = cv2.INTER_AREA)
#     #real_kalman_image = cv2.resize(estimage_path_image3, None, fx = 0.25, fy = 0.25, interpolation = cv2.INTER_AREA)

#     text6 = "Robust Template Matching"
#     cv2.putText(real_measure_image, text6, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 2, cv2.LINE_AA)
    
#     #cv2.imshow("real_measure_image",real_measure_image)
#     #cv2.imshow("real_kalman_image", real_kalman_image)

#     #################

#     XX = int(xxx)
#     YY = int(yyy)

#     XXX = round((XX - 1486 +7 ) * 4.810943396 / 1000, 3)
#     YYY = round((YY - 1335 -2 ) * 4.810943396 / 1000, 3)

#     # ######실제와 칼만 필터 사이의 오차#####

#     # esti_XX = int(x_pos_esti)
#     # esti_YY = int(y_pos_esti)

#     # esti_XXX = round((esti_XX - 1486 + 7) * 4.810943396 / 1000, 3)
#     # esti_YYY = round((esti_YY - 1335 - 2) * 4.810943396 / 1000, 3)

#     # # cv2.putText(QWERT1, str(esti_XXX), (180,40), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2, cv2.LINE_AA)
#     # # cv2.putText(QWERT1, str(esti_YYY), (180,80), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2, cv2.LINE_AA)

#     # # cv2.putText(QWERT1, "m", (280,40), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2, cv2.LINE_AA)
#     # # cv2.putText(QWERT1, "m", (280,80), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2, cv2.LINE_AA)

#     center_position[0] += 5.485
#     center_position[1] -= 0.0245

#     # error_esti_xx = abs(abs(center_position[0]) - esti_XXX)
#     # error_esti_yy = abs(abs(center_position[1]) - esti_YYY)

#     # error_kalman_x.append(error_esti_xx)
#     # error_kalman_y.append(error_esti_yy)

    
#     # print("___________________________________________________",error_esti_xx)
#     # print("___________________________________________________",error_esti_yy)

#     # ###### 실제와 탬플릿 매칭 사이의 오차 ########

#     error_xx = abs(abs(center_position[0]) - XXX)
#     error_yy = abs(abs(center_position[1]) - YYY)

#     error_x.append(error_xx)
#     error_y.append(error_yy)
    
#     text_XXX = "X axis : "+str(XXX)+"m"
#     text_YYY = "Y axis : "+str(YYY)+"m"
#     text_X = "x"
#     text_Y = "y"

#     print("________________Robust Template Matching x error___________________",error_xx)
#     print("________________Robust Template Matching y error___________________",error_yy)

#     cv2.putText(real_measure_image, text_XXX, (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 2, cv2.LINE_AA)
#     cv2.putText(real_measure_image, text_YYY, (10,120), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 2, cv2.LINE_AA)
#     cv2.putText(real_measure_image, text_X, (348,232), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,255), 2, cv2.LINE_AA)
#     cv2.putText(real_measure_image, text_Y, (238,322), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,255,0), 2, cv2.LINE_AA)
#     cv2.arrowedLine(real_measure_image, (268,242), (348,242), (0,0,255), 4)
#     cv2.arrowedLine(real_measure_image, (268,242), (268,322), (0,255,0), 4)
#     #cv2.circle(real_measure_image,(268,242), 10,(0,0,255),10)

#     cv2.imshow("real_measure_image",real_measure_image)
#     ##################### 10 cm 안으로 몇개 들어왓는지 확인####################
#     ###### 실제와 탬플릿 매칭 ######
#     if 0 <= error_xx <= 0.1 and 0<= error_yy <= 0.1:
#         n +=1

#     k+=1

#     ###### 실제와 칼만필터 ######

#     if 0 <= error_esti_xx_k <= 0.1 and error_esti_yy_k <= 0.1:
#         o1 +=1
    
#     # ###################################################################

#     ##### 매칭된 정보가 올바른지에 대한 판단 #####
#     diff_time =  time.time() - second

#     time_time = (time_list[-1] + diff_time)
#     time_time1 = round(time_time, 3)
#     time_list.append(time_time1)

#     key = cv2.waitKey(1)
    
#     if key == 27:
#         cv2.destroyAllWindows()
#         break

#     elif key == ord('c'):
       
#         # cv2.imwrite("image/123.png", final)
#         # cv2.imwrite("image/456.png", matching_final)

#         cv2.destroyAllWindows()

# # print(error_x)
# # print(error_y)
# # print("###################")
# # print(error_kalman_x)
# # print(error_kalman_y)
# # print("###################")
# # print(real_x)
# # print(real_y)

# ####### ###### 실제와 탬플릿 매칭 ######

# # print(time_list)

# print(k)
# print(n)
# print(o1)

# percent_matching = (n/k) * 100

# percent_kalman = (o1/k) * 100

# print("10cm_matching : ",percent_matching)
# print("10cm_kalman : ", percent_kalman)

# matching_x_sum = sum(error_x)
# matching_y_sum = sum(error_y)

# kalman_x_sum = sum(error_kalman_x_k)
# kalman_y_sum = sum(error_kalman_y_k)

# mean_matching_x = matching_x_sum / n
# mean_matching_y = matching_y_sum / n

# mean_kalman_x = kalman_x_sum / n
# mean_kalman_y = kalman_y_sum / n

# print("matching x mean :",mean_matching_x)
# print("matching y mean :",mean_matching_y)

# print("kalman x mean :", mean_kalman_x)
# print("kalman y mean :", mean_kalman_y)

# text1 = open("data/error_x.pkl", "wb")
# pickle.dump(error_x, text1)
# text1.close()

# text2 = open("data/error_y.pkl", "wb")
# pickle.dump(error_y, text2)
# text2.close()

# text3 = open("data/error_kalman_x_k.pkl", "wb")
# pickle.dump(error_kalman_x_k, text3)
# text3.close()

# text4 = open("data/error_kalman_y_k.pkl", "wb")
# pickle.dump(error_kalman_y_k, text4)
# text4.close()

# text5 = open("data/time_list.pkl", "wb")
# pickle.dump(time_list, text5)
# text5.close()

# # plt.figure(figsize=(10,10))
# # plt.plot(time_list, error_x, c = 'r')
# # plt.xlabel('time [sec]')
# # plt.ylabel('measure x error [mm]')
# # plt.title('time-measure(x) error')

# # plt.figure(figsize=(10,10))
# # plt.plot(time_list, error_y, c = 'r')
# # plt.xlabel('time [sec]')
# # plt.ylabel('measure y error [mm]')
# # plt.title('time-measure(y) error')

# # plt.figure(figsize=(10,10))
# # plt.plot(time_list, error_kalman_x_k, c = 'r')
# # plt.xlabel('time[sec]')
# # plt.ylabel('kalman x error [mm]')
# # plt.title('time-kalman(x)_k error')

# # plt.figure(figsize=(10,10))
# # plt.plot( time_list, error_kalman_y_k, c = 'r')
# # plt.xlabel('time[sec]')
# # plt.ylabel('kalman y error [mm]')
# # plt.title('time-kalman(y)_k error')

# plt.figure(figsize=(10,10))
# plt.plot(time_list, error_x, c = 'r')
# plt.plot(time_list, error_kalman_x_k, c = 'b')
# plt.xlabel('time [sec]')
# plt.ylabel('x error [m]')
# plt.title('time-measure(x) error')

# plt.figure(figsize=(10,10))
# plt.plot(time_list, error_y, c = 'r')
# plt.plot( time_list, error_kalman_y_k, c = 'b')
# plt.xlabel('time [sec]')
# plt.ylabel('measure y error [m]')
# plt.title('time-measure(y) error')

# plt.show()

