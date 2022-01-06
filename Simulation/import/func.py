#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sim
import cv2
import sympy as sp
import numpy as np
import time
import array
import math
import imutils
from random import *
from PIL import Image as I
from numpy.linalg import inv
from skimage.transform import (hough_line, hough_line_peaks)

############################################################################################

def connect(port):
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0: print(port, "포트 연결 성공")
    else: print(port, "포트 연결 실패")
    return clientID

############################################################################################

#이미지 BGR to RGB
def bgr_to_rgb(image):
    src = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return src

############################################################################################

def rotate(image, degree, num):
    src = image 
    if num == 0: # color
        height, width, channel = src.shape
    if num == 1: # gray
        height, width = src.shape
    #print(height)
    #print(width)
    matrix = cv2.getRotationMatrix2D((width/2, height/2), degree, 1)
    dst = cv2.warpAffine(src, matrix, (width, height))
    return dst

############################################################################################

#영상 반전
#상하 반전 → number = 0
#좌우 반전 → number = 1
def flip(image, number):
    src = cv2.flip(image, number)
    return src

############################################################################################

def mouse_callback(event, x, y, flags, param): 
    print("마우스 이벤트 발생, x:", x ," y:", y) # 이벤트 발생한 마우스 위치 출력
    
############################################################################################

def roi_point(x, y, distance, degree):
    radian = degree * (180 / math.pi)
    
    new_x = x + (distance * math.cos(radian))
    new_y = y + (distance * math.sin(radian))
    
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

############################################################################################

def point_extraction(x1, y1, x2, y2):

    width = x2 - x1
    height = y2 - y1

    center_x = x1 + (x2 - x1)/2
    center_y = y1 + (y2 - y1)/2

    pt1 = []
    pt2 = []

    pt2.append([center_x, center_y])
    pt1.append([center_x - (width / 2),center_y - (height/2)])    
    pt1.append([center_x + (width / 2), center_y - (height/2)])
    pt1.append([center_x - (width / 2), center_y + (height/2)])
    pt1.append([center_x + (width / 2), center_y + (height/2)])

    return pt1, pt2

############################################################################################

def rotate_rectangle(x1, y1, x2, y2, alpha):

    v_1 = x1 - x2
    v_2 = y1 - y2

    angle = alpha * math.pi / 180

    rot1 = v_1*math.cos(angle) - v_2*math.sin(angle)
    rot2 = v_1*math.sin(angle) + v_2*math.cos(angle)

    result1 = abs(rot1 + x2)
    result2 = abs(rot2 + y2)

    return int(result1), int(result2)

############################################################################################

def draw_rotate_rectangel(image, a, b, c, d, number, width):
    if number == 1:
        color = (0, 0, 255)
    elif number == 2:
        color = (0, 255, 0)
    elif number == 3:
        color = (255, 0, 0)
        
    draw_image = cv2.line(image, a, b, color, width)
    draw_image = cv2.line(draw_image, a, c, color, width)
    draw_image = cv2.line(draw_image, c, d, color, width)
    draw_image = cv2.line(draw_image, d, b, color, width)
    
    return draw_image

############################################################################################

def degree2radian(degree):
    radian = degree * (math.pi / 180)
    return radian

###########################################################################################

def result_point(x, y, resol_x, resol_y, degree):

    theta = degree2radian(degree)
    Trans = np.array([[math.cos(theta),-math.sin(theta)], [math.sin(theta), math.cos(theta)]])

    a = x - resol_x
    b = y - resol_y

    initial = np.array([[a], [b]])

    result = np.dot(Trans, initial) + np.array([[resol_x], [resol_y]])
    return result

###########################################################################################

def kalman_filter(z_meas, x_esti, P):
    """Kalman Filter Algorithm."""

    dt = 1

    A = np.array([[ 1, dt,  0,  0],
              [ 0,  1,  0,  0],
              [ 0,  0,  1, dt],
              [ 0,  0,  0,  1]])

    H = np.array([[ 1,  0,  0,  0],
                [ 0,  0,  1,  0]])

    Q = 1.0 * np.eye(4)  #기본 

    
    R1 =  np.array([[50,  0],[ 0, 50]]) * 1000 # big
    R2 =  np.array([[50,  0],[ 0, 50]])

    #R2 = np.array([[50,  0],[ 0, 50]]) * 10000
    R3 = np.array([[50,  0],[ 0, 50]]) / 1000 # small

    # if abs(abs(z_meas[0]) - abs(x_esti[0])) >= 150 or abs(abs(z_meas[1]) - abs(x_esti[2])) >= 150 :
    #     R = R1 * 1000
    #     Q /= 1000

    #     # (1) Prediction.
    #     #x_pred = A @ x_esti
    #     x_pred = np.dot(A, x_esti)

    #     #P_pred = A @ P @ A.T + Q
    #     A1 = np.dot(A, P)
    #     A2 = np.transpose(A)
    #     P_pred = np.dot(A1, A2) + Q
        
    #     # (2) Kalman Gain.
    #     #K = P_pred @ H.T @ inv(H @ P_pred @ H.T + R)
    #     B1 = np.transpose(H)
    #     B2 = np.dot(P_pred, B1)
    #     B3 = np.dot(H, P_pred)
    #     B4 = np.dot(B3, B1) + R
    #     B5 = inv(B4)
    #     K = np.dot(B2, B5) / 10000

    #     # (3) Estimation.
    #     #x_esti = x_pred + K @ (z_meas - H @ x_pred)
    #     C1 = np.dot(H, x_pred)
    #     C2 = z_meas - C1
    #     C3 = np.dot(K, C2)
    #     x_esti = x_pred + C3

    #     # (4) Error Covariance.
    #     #P = P_pred - K @ H @ P_pred
    #     D1 = np.dot(K, H)
    #     D2 = np.dot(D1, P_pred)
    #     P = P_pred - D2

        

    # else:
    #     R = R2 / 10
    #     Q = Q * 10


    #     # (1) Prediction.
    #     #x_pred = A @ x_esti
    #     x_pred = np.dot(A, x_esti)

    #     #P_pred = A @ P @ A.T + Q
    #     A1 = np.dot(A, P)
    #     A2 = np.transpose(A)
    #     P_pred = np.dot(A1, A2) + Q
        
    #     # (2) Kalman Gain.
    #     #K = P_pred @ H.T @ inv(H @ P_pred @ H.T + R)
    #     B1 = np.transpose(H)
    #     B2 = np.dot(P_pred, B1)
    #     B3 = np.dot(H, P_pred)
    #     B4 = np.dot(B3, B1) + R
    #     B5 = inv(B4)
    #     K = np.dot(B2, B5) 

    #     # (3) Estimation.
    #     #x_esti = x_pred + K @ (z_meas - H @ x_pred)
    #     C1 = np.dot(H, x_pred)
    #     C2 = z_meas - C1
    #     C3 = np.dot(K, C2)
    #     x_esti = x_pred + C3

    #     # (4) Error Covariance.
    #     #P = P_pred - K @ H @ P_pred
    #     D1 = np.dot(K, H)
    #     D2 = np.dot(D1, P_pred)
    #     P = P_pred - D2

    # return x_esti, P

    R = R2 
    Q = Q


    # (1) Prediction.
    #x_pred = A @ x_esti
    x_pred = np.dot(A, x_esti)

    #P_pred = A @ P @ A.T + Q
    A1 = np.dot(A, P)
    A2 = np.transpose(A)
    P_pred = np.dot(A1, A2) + Q
    
    # (2) Kalman Gain.
    #K = P_pred @ H.T @ inv(H @ P_pred @ H.T + R)
    B1 = np.transpose(H)
    B2 = np.dot(P_pred, B1)
    B3 = np.dot(H, P_pred)
    B4 = np.dot(B3, B1) + R
    B5 = inv(B4)
    K = np.dot(B2, B5) 

    # (3) Estimation.
    #x_esti = x_pred + K @ (z_meas - H @ x_pred)
    C1 = np.dot(H, x_pred)
    C2 = z_meas - C1
    C3 = np.dot(K, C2)
    x_esti = x_pred + C3

    # (4) Error Covariance.
    #P = P_pred - K @ H @ P_pred
    D1 = np.dot(K, H)
    D2 = np.dot(D1, P_pred)
    P = P_pred - D2

    return x_esti, P

################################

def kalman_filter_k(z_meas, x_esti, P):
    """Kalman Filter Algorithm."""

    dt = 1

    A = np.array([[ 1, dt,  0,  0],
              [ 0,  1,  0,  0],
              [ 0,  0,  1, dt],
              [ 0,  0,  0,  1]])

    H = np.array([[ 1,  0,  0,  0],
                [ 0,  0,  1,  0]])

    Q = 0.65 * np.eye(4)  #기본 

    
    R1 =  np.array([[50,  0],[ 0, 50]]) * 1000 # big
    R2 =  np.array([[9,  0],[ 0, 9]])

    #R2 = np.array([[50,  0],[ 0, 50]]) * 10000
    R3 = np.array([[50,  0],[ 0, 50]]) / 1000 # small

    R = R2
    Q = Q 


    # (1) Prediction.
    #x_pred = A @ x_esti
    x_pred = np.dot(A, x_esti)

    #P_pred = A @ P @ A.T + Q
    A1 = np.dot(A, P)
    A2 = np.transpose(A)
    P_pred = np.dot(A1, A2) + Q
    
    # (2) Kalman Gain.
    #K = P_pred @ H.T @ inv(H @ P_pred @ H.T + R)
    B1 = np.transpose(H)
    B2 = np.dot(P_pred, B1)
    B3 = np.dot(H, P_pred)
    B4 = np.dot(B3, B1) + R
    B5 = inv(B4)
    K = np.dot(B2, B5) 

    # (3) Estimation.
    #x_esti = x_pred + K @ (z_meas - H @ x_pred)
    C1 = np.dot(H, x_pred)
    C2 = z_meas - C1
    C3 = np.dot(K, C2)
    x_esti = x_pred + C3

    # (4) Error Covariance.
    #P = P_pred - K @ H @ P_pred
    D1 = np.dot(K, H)
    D2 = np.dot(D1, P_pred)
    P = P_pred - D2

    return x_esti, P
