#! /usr/bin/env python
#-*- encoding: utf-8 -*-

import rospy
import numpy as np
import math
import pickle
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from openrobot_vesc_msgs.msg import *
from std_msgs.msg import *

#############################
count = "11M_24D_025"
#############################

class Visualiser:

    stamp = 0

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'r', linewidth=2, label="Wheel Velocity Odometry")

        self.x_pos_data, self.y_pos_data = [], []

    def plot_init(self):
        
        self.ax.set_xticks([-4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0]) # plt range x value
        self.ax.set_yticks([-1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4.0]) # plt range y value

        # self.ax.set_xlim(-100, 100)
        # self.ax.set_ylim(-100, 100)

        return self.ln

    def pose_callback(self, msg):

        global dt
        global gear_ratio
        global odometry_info

        global x_plt_data
        global y_plt_data

        global u1_speed
        global u2_speed
        global u3_speed
        global u4_speed

        ####### rostopic subscribe about each wheel data #######

        u1 = - float(msg.diff_deg_now[0]) / gear_ratio / (1.26/1.24) #encoder can id = 1
        u2 = float(msg.diff_deg_now[1]) / gear_ratio / (1.26/1.24)#encoder can id = 4
        u3 = float(msg.diff_deg_now[3]) / gear_ratio / (1.26/1.24)#encoder can id = 2
        u4 = float(msg.diff_deg_now[2]) / gear_ratio / (1.26/1.24)#encoder can id = 3

        ####### essential data for odometry compute #######

        odometry_info = self.odometry(u1, u2, u3, u4, odometry_info, dt)

        real_x_pos = odometry_info[1]
        real_y_pos = odometry_info[2]

        ####### append data #######

        # (1) All data (stamp, each wheel data, odometry_result)

        wheel_speed = np.array([int(self.stamp) ,u1, u2, u3, u4, -real_y_pos, real_x_pos]) # unit : radian / second
        wheel_data.append(wheel_speed)

        # (2) Each wheel data ------> unit : radain / second

        u1_speed.append(u1)
        u2_speed.append(u2)
        u3_speed.append(u3)
        u4_speed.append(u4)

        # (3) plt data
        self.x_pos_data.append(-real_y_pos)
        self.y_pos_data.append(real_x_pos)

        # (4) wheel odometry result data
        
        x_plt_data.append(-real_y_pos)
        y_plt_data.append(real_x_pos)

        ###### print data #######

        # (1) stamp 
        # print("stamp = {}".format(self.stamp))

        # (2) wheel velocity ---> unit : radain / second

        # print("u1 = {}".format(u1))
        # print("u2 = {}".format(u2))
        # print("u3 = {}".format(u3))
        # print("u4 = {}".format(u4))

        # (3) wheel velocity ---> unit : degree / second

        deg2rad = lambda deg: deg/(np.pi/180)

        # u1_deg = deg2rad(u1) 
        # u2_deg = deg2rad(u2)
        # u3_deg = deg2rad(u3)
        # u4_deg = deg2rad(u4)

        # print('can ID : {} --> dps : {}'.format(msg.can_id[0], u1_deg))
        # print('can ID : {} --> dps : {}'.format(msg.can_id[1], u2_deg))
        # print('can ID : {} --> dps : {}'.format(msg.can_id[3], u3_deg))
        # print('can ID : {} --> dps : {} \n'.format(msg.can_id[2], u4_deg))

        # (4) wheel odometry result 

        print("x_pos : {} m".format(-real_y_pos))
        print("y_pos : {} m".format(real_x_pos))

    
        print("#########################################")


        ####### etc #######

        # (1) publish data

        wheel_odometry = Float64MultiArray()

        wheel_odometry.data = np.array([real_x_pos, real_y_pos])

        pub.publish(wheel_odometry)

        # (2) stamp 

        self.stamp +=1


    def update_plot(self, frame):
        self.ln.set_data(self.x_pos_data, self.y_pos_data)
        return self.ln

    def odometry(self, u1, u2, u3, u4, odometry_info, dt):
        
        ######## kinematic model of mecanum mobile platform ########

        r = 0.071 #0.0867(3.2m), #0.0813(3.0m)
        l = 0.23
        w = 0.25225
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

        # print(vbx)
        # print(vby)
        
        twist = np.dot(F, u)
        w_b, v_x, v_y = twist

        robot_velocity = Float64MultiArray()

        robot_velocity.data = np.array([v_x])

        pub1.publish(robot_velocity)

        print("robot_x_speed : {}".format(v_x))
        # print("robot_y_speed : {}".format(v_y))


        ######## {b} coordinate --> {s} coordinate about d_qb ########

        T_sb = np.array([[1.0, 0.0, 0.0],[0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])

        if  wbz == 0:
            d_qb = np.array([0, vbx, vby])
        
        else:
            d_qb = np.array([wbz, 
                            (vbx * math.sin(wbz) + vby * (math.cos(wbz) - 1))/wbz, 
                            (vby * math.sin(wbz) + vbx * (1 - math.cos(wbz)))/wbz])

        d_q = np.dot(T_sb, d_qb)


        ######## odometry information update about timestep t ########

        phi_update = phi + d_q[0]
        x_pos_update = x_pos + d_q[1]
        y_pos_update = y_pos + d_q[2]
        new_wheel_ang = wheel_ang + delta_wheel_ang

        ####### print data #######

        # print("phi = {}".format(phi_update))
        # print("final_x_pos = {}".format(x_pos_update))
        # print("final_y_pos = {}".format(y_pos_update))
        # print("----------------------------------------")
        

        ####### result ########
        update_odometry_info = np.r_[phi_update, x_pos_update, y_pos_update, new_wheel_ang]

        return update_odometry_info


######## init value ########

phi, x_pos, y_pos = 0.0, 0.0, 0.0
w1, w2, w3, w4 = 0.0, 0.0, 0.0, 0.0
odometry_info = np.r_[np.array([phi, x_pos, y_pos, w1, w2, w3, w4])]

Hz = 200 #50
dt = 1./Hz # degreee difference in 0.02
gear_ratio = 14.0

x_plt_data = []
y_plt_data = []

u1_speed = []
u2_speed = []
u3_speed = []
u4_speed = []

wheel_data = []

######## ROS node #########

rospy.init_node('odometry_msg')
rospy.Rate(200)
vis = Visualiser()
sub = rospy.Subscriber("/dev/ttyACM0/sensors/customs", VescGetCustomApp, vis.pose_callback)
pub = rospy.Publisher("/odometry/wheel_velocity", Float64MultiArray, queue_size= 1)

pub1 = rospy.Publisher("/odometry/robot_velocity", Float64MultiArray, queue_size= 1)

######## show plotting #########

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.legend(loc= 'upper left')
plt.show()

######## save pickle data ########

# (1) wheel odometry result data
with open("/home/jetson/workspace/catkin_ws_localization/src/data/pose/x_data_{}.pickle".format(count), "wb") as fw:
    pickle.dump(x_plt_data, fw)
with open("/home/jetson/workspace/catkin_ws_localization/src/data/pose/y_data_{}.pickle".format(count), "wb") as fw:
    pickle.dump(y_plt_data, fw)

# (2) Each wheel data
with open("/home/jetson/workspace/catkin_ws_localization/src/data/wheel_velocity/u1_data_{}.pickle".format(count), "wb") as u1:
    pickle.dump(u1_speed, u1)

with open("/home/jetson/workspace/catkin_ws_localization/src/data/wheel_velocity/u2_data_{}.pickle".format(count), "wb") as u2:
    pickle.dump(u2_speed, u2)

with open("/home/jetson/workspace/catkin_ws_localization/src/data/wheel_velocity/u3_data_{}.pickle".format(count), "wb") as u3:
    pickle.dump(u3_speed, u3)

with open("/home/jetson/workspace/catkin_ws_localization/src/data/wheel_velocity/u4_data_{}.pickle".format(count), "wb") as u4:
    pickle.dump(u4_speed, u4)

# (3) All data(stamp, each wheel data, odometry result)
with open("/home/jetson/workspace/catkin_ws_localization/src/data/all_wheel_data_{}.pickle".format(count), "wb") as all:
    pickle.dump(wheel_data, all)