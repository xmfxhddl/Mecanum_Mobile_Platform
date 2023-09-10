#! /usr/bin/env python3
#-*- encoding: utf-8 -*-

from filterpy.kalman import KalmanFilter
import numpy as np
import math

def MecanumKalman(A, B, u, z):
    global n, Q_para, R_para, past_pose
    global x, P, Q, H, R
    
    if n == 0:
        H = np.eye(3)

        ####### Experimental value through trial and error #######
        
        Q = Q_para * np.eye(3)
        R = R_para * np.eye(3)

        x = np.array([[0, 0, 0]]).T # initial value
        past_pose = np.array([[0, 0, 0]]).T
        P = 1 * np.eye(3)
        
        n += 1
        
    ###### {b} ----> [s] #######
    
    V_b = B@u

    w_bz = V_b[0][0]
    v_bx = V_b[1][0]
    v_by = V_b[2][0]
    
    if w_bz == 0:
        d_qb = np.array([[v_bx], 
                      [v_by], 
                      [0]], dtype=object)
    else:
        d_qb = np.array([[(v_bx * math.sin(w_bz) + v_by * (math.cos(w_bz) - 1))/w_bz], 
                        [(v_by * math.sin(w_bz) + v_bx * (1 - math.cos(w_bz)))/w_bz], 
                        [w_bz]], dtype=object)
    
    T_sb = np.array([[math.cos(d_qb[2][0]), -math.sin(d_qb[2][0]), 0.0],
                    [math.sin(d_qb[2][0]), math.cos(d_qb[2][0]), 0.0], 
                    [0, 0, 1]])
 
    d_q = T_sb @ d_qb
    
    d_trans_q = np.array([[-d_q[1][0]],[d_q[0][0]],[d_q[2][0]]])
    
    # real_pos = A @ past_pose + (d_trans_q*1000)
    # past_pose = real_pos
    
    ###### kalman filter ######
    
    x_pred = A@x + (d_trans_q * 1000) # x_pred = A@x + (d_q * 1000)  
    P_pred = A@P@A.T + Q
    
    K = P_pred@H.T @ (np.linalg.inv(H @ P_pred @ H.T + R))

    x = x_pred + K@(z - H@x_pred)
    P = P_pred - K@H@P_pred
    
    ###### print data ######
    
    # print("prediction state value = \n{} →→→ 3x1".format(x_pred.shape))
    # print("error covariance matrix prediction = \n{} →→→ 3x3".format(P_pred.shape))
    # print("Kalman gain = \n{} →→→ 3x3".format(K.sahpe))
    # print("Estimation value = \n{} →→→ 3x1".format(x.shape))
    
    pos_x = x[0]
    pos_y = x[1]
    theta = x[2]
    
    return pos_x, pos_y #past_pose[0], past_pose[1]

x_pos_final_list = []
y_pos_final_list = []

real_pose_x = []
real_pose_y = []

n = 0

r = 0.0762
l = 0.23
w = 0.25225

Q_para = 0.0001 #0.05
R_para = 5000 #100

Hz = 250
dt = 1/Hz
alpha = l + w

F = r/4 * np.array([[-1/alpha, 1/alpha, 1/alpha, -1/alpha], [1, 1, 1, 1], [-1, 1, -1, 1]])

A = np.eye(3)
B = F *dt

count = len(list_x_wh_ang)

for i in range (0, count):
    z = np.array([[list_x_vi[i]], [list_y_vi[i]], [list_theta_vi[i]]], dtype=object)
#     print("z = {}".format(z))
    u = np.array([[w1_angle_data[i]],[w2_angle_data[i]],[w3_angle_data[i]],[w4_angle_data[i]]], dtype=object)
#     print("u = {} \n".format(u))
    x_pos_final, y_pos_final, real_pose_xx, real_pose_yy = MecanumKalman(A, B, u, z)
    
    x_pos_final_list.append(x_pos_final)
    y_pos_final_list.append(y_pos_final)
    
    real_pose_x.append(real_pose_xx)
    real_pose_y.append(real_pose_yy)
    
#     print("x = {} \n y = {}\n".format(x_pos_final, y_pos_final))
    
# plt.xlim(-3300, 100.0)
# plt.ylim(-100, 3300.0)

print("KALMAN : x = {} \n         y = {}\n".format(x_pos_final[-1], y_pos_final[-1]))
print("VISUAL : x = {} \n         y = {}\n".format(list_x_vi[-1], list_y_vi[-1]))
print("WHEEL  : x = {} \n         y = {}\n".format(list_x_wh_ang[-1], list_y_wh_ang[-1]))

plt.plot(x_pos_final_list, y_pos_final_list, c='c')
plt.plot(list_x_vi, list_y_vi, c='r')
plt.plot(list_x_wh_ang, list_y_wh_ang, c='b')
# plt.plot(real_pose_x[0:5000], real_pose_y[0:5000], c='b')