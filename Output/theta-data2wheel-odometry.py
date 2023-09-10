import pickle
import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

# Fixed parameter
gear_ratio = 14.0 # gear_ratio: 기어비 14:1
r = 0.0762 # r: 바퀴의 반지름
l = 0.23 # l: 메카넘 모바일 플랫폼의 가로 길이
w = 0.25225 # w: 메카넘 모바일 플랫폼의 세로 길이
alpha = l + w

# Define Initial parameter

i = 0

Hz = 50
dt = 1./Hz # degreee difference in 0.02
phi, x_pos, y_pos = 0.0, 0.0, 0.0
w1, w2, w3, w4 = 0.0, 0.0, 0.0, 0.0
odometry_info = np.r_[np.array([phi, x_pos, y_pos, w1, w2, w3, w4])]

dt = 1.0/50 # degreee difference in 0.1ms(10kHz)

def convert_wheel_velocity(all_encoder_data, n):
    
    global i
    global dt
    
    if i == 0:
        global odometry_info
        
        u1 = all_encoder_data[n][0]
        u2 = all_encoder_data[n][1]
        u3 = all_encoder_data[n][2]
        u4 = all_encoder_data[n][3]
     
        # u1 = rad2deg(all_encoder_data[n][0])
        # u2 = rad2deg(all_encoder_data[n][1])
        # u3 = rad2deg(all_encoder_data[n][2])
        # u4 = rad2deg(all_encoder_data[n][3])
        
        odometry_info, real_x_pos, real_y_pos = odometry(u1, u2, u3, u4, odometry_info, dt)
        
        print("x_pos : {} m".format(real_x_pos))
        print("y_pos : {} m".format(real_y_pos))
        
        i += 1
        
        return odometry_info, real_x_pos, real_y_pos
        
    
    else :

        u1 = all_encoder_data[n][0]
        u2 = all_encoder_data[n][1]
        u3 = all_encoder_data[n][2]
        u4 = all_encoder_data[n][3]

        odometry_info, real_x_pos, real_y_pos = odometry(u1, u2, u3, u4, odometry_info, dt)
        print("x_pos : {} m".format(real_x_pos))
        print("y_pos : {} m".format(real_y_pos))
        
        return odometry_info, real_x_pos, real_y_pos
        
    
def deg2rad(degree):
    
    rad = degree * np.pi / 180

    return rad

def odometry(u1, u2, u3, u4, odometry_info, dt):
    
    ######## kinematic model of mecanum mobile platform ########

    r = 0.0762
    l = 0.23
    w = 0.25225  # 0.22805 + 0.045/2
    alpha = l + w

    F = r/4 * np.array([[-1/alpha, 1/alpha, 1/alpha, -1/alpha], [1, 1, 1, 1], [-1, 1, -1, 1]])
    
    ######## odometry information update about timestep t-1 ########

    phi = odometry_info[0]
    x_pos = odometry_info[1]
    y_pos = odometry_info[2]
    wheel_ang = odometry_info[3:7]

    ######## calculate delta_wheel_ang about timestep t ########

    u = np.array([u1, u2, u3 ,u4])
    delta_wheel_ang = u * dt

    ######## calculate chassis velocity ########

    twist_b = np.dot(F, delta_wheel_ang)
    wbz, vbx, vby = twist_b

    ######## {b} coordinate --> {s} coordinate about d_qb ########

    T_sb = np.array([[1.0, 0.0, 0.0],[0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])

    if wbz == 1e-4:
        d_qb = np.array([0, vbx, vby])
    
    else:
        d_qb = np.array([wbz, (vbx * math.sin(wbz) + vby * (math.cos(wbz) - 1))/wbz, (vby * math.sin(wbz) + vbx * (1 - math.cos(wbz)))/wbz])

    d_q = np.dot(T_sb, d_qb)

    ######## odometry information update about timestep t ########

    phi_update = phi + d_q[0]
    x_pos_update = x_pos + d_q[1]
    y_pos_update = y_pos + d_q[2]
    new_wheel_ang = wheel_ang + delta_wheel_ang

    update_odometry_info = np.r_[phi_update, x_pos_update, y_pos_update, new_wheel_ang]

    return update_odometry_info, x_pos_update, y_pos_update

with open('./data/theta_data.pickle', 'rb') as th:
    all_encoder_data = pickle.load(th, encoding='latin1')
    
list_count = len(all_encoder_data)
print(list_count)

for i in range(0, 50):
    odometry_info, output_x, output_y = convert_wheel_velocity(all_encoder_data, i)
