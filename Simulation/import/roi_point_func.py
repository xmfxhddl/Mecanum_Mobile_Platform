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
from func import *
from PIL import Image as I
from skimage.transform import (hough_line, hough_line_peaks)


################### [LEFT]
def roi_point_left2right(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree

    radian = (math.pi/180) * (degree)
    
    real_distance = distance / 2

    new_x = x + (real_distance * math.cos(radian))
    new_y = y + (real_distance * math.sin(radian))
    print(new_x)
    print(new_y)
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

def roi_point_left2front(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree
    
    radian = (math.pi/180) * (degree - 45)

    new_x = x + (distance * math.cos(radian))
    new_y = y + (distance * math.sin(radian))

    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

def roi_point_left2back(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree
    
    radian = (math.pi/180) * (degree + 45)
    
    new_x = x + (distance * math.cos(radian))
    new_y = y + (distance * math.sin(radian))
    
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

################################## [RIGHT] #########################

def roi_point_right2front(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree
    
    radian = (math.pi/180) * (degree - 135)
    
    new_x = x + (distance * math.cos(radian))
    new_y = y + (distance * math.sin(radian))
    
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

def roi_point_right2back(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree
    
    radian = (math.pi/180) * (degree + 135)
    
    new_x = x + (distance * math.cos(radian))
    new_y = y + (distance * math.sin(radian))
    
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

def roi_point_right2left(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree
    
    radian = (math.pi/180) * (degree - 180)
    
    real_distance = distance / 2

    new_x = x + (real_distance * math.cos(radian))
    new_y = y + (real_distance * math.sin(radian))
    
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

##################################### [Front] ##################################

def roi_point_front2back(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree +180
    
    radian = (math.pi/180) * (degree + 90)
    
    real_distance = distance / 2

    new_x = x + (real_distance * math.cos(radian))
    new_y = y + (real_distance * math.sin(radian))
    print(new_y)
    
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 

#################################### [Back] #####################################

def roi_point_back2front(x, y, distance, degree):
    
    if -90 < degree and degree <= 0:
        degree = - degree 
    elif degree == -90:
        degree = degree - 180 
    elif -180 < degree and degree < -90:
        degree = - degree
    elif degree == -180:
        degree = degree
    elif 0 < degree and degree <= 360:
        degree = -degree
    
    radian = (math.pi/180) * (degree - 90)
    
    real_distance = distance / 2

    new_x = x + (real_distance * math.cos(radian))
    new_y = y + (real_distance * math.sin(radian))
    
    new_x = int(new_x)
    new_y = int(new_y)
    
    return new_x, new_y 
