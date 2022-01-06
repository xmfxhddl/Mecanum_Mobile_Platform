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

################################
count = 0.5
################################

# wheel odometry using wheel position + plt plotting
class Visualiser:

    stamp = 0

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'b', linewidth=2, label="Wheel Position Odometry")

        self.x_pos_data, self.y_pos_data = [], []

    def plot_init(self):
        
        # self.ax.set_xlim(-6, 6) # plt limit x value
        # self.ax.set_ylim(-6, 6) # plt limit y value

        # self.ax.set_aspect('equal', adjustable = 'box')
        self.ax.set_xticks([-4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0]) # plt range x value
        self.ax.set_yticks([-1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4.0])

        return self.ln

    def pose_callback(self, msg):
        
        global i

        global dt
        global gear_ratio

        global x_plt_data
        global y_plt_data

        deg2rad = lambda deg: deg*np.pi/180

        ###### wheel position ######
        if i == 0:
            
            ###### rostopic subscibe about each wheel position #######

            theta1 = -(float(msg.accum_deg_now[0]) * 100) / gear_ratio
            theta2 = float(msg.accum_deg_now[1]) / gear_ratio
            theta3 = float(msg.accum_deg_now[3]) / gear_ratio
            theta4 = float(msg.accum_deg_now[2]) / gear_ratio

            ####### initial value setting #######

            phi, x_pos, y_pos = 0.0, 0.0, 0.0

            ####### essential data for odometry compute ########
            
            #                               0      1     2       3       4       5       6
            self.odometry_info = np.array([phi, x_pos, y_pos, theta1, theta2, theta3, theta4])

            real_x_pos = self.odometry_info[1]
            real_y_pos = -self.odometry_info[2]

            ####### append data #######
            
            # (1) plt data
            self.x_pos_data.append(real_y_pos)
            self.y_pos_data.append(real_x_pos)

            # (2) wheel odometry result data 
            x_plt_data.append(real_y_pos)
            y_plt_data.append(real_x_pos)

            # (3) wheel position data

            theta1_pos.append(theta1)
            theta2_pos.append(theta2)
            theta3_pos.append(theta3)
            theta4_pos.append(theta4)

            # (4) All data (stamp, each wheel position, odometry result)

            wheel_position = np.array([self.stamp, theta1, theta2, theta3, theta4, real_y_pos, real_x_pos])
            wheel_data.append(wheel_position)

            ####### print data #######

            print('--------------------------------')

            # (1) stamp
            print("stamp = {}".format(self.stamp))

            # (2) wheel position (unit : radian)
            print('can ID : {} --> radian : {}'.format(msg.can_id[0], theta1))
            print('can ID : {} --> radian : {}'.format(msg.can_id[1], theta2))
            print('can ID : {} --> radian : {}'.format(msg.can_id[3], theta3))
            print('can ID : {} --> radian : {}'.format(msg.can_id[2], theta4))

            # (3) wheel position (unit : degree)

            # theta1_degree = deg2rad(theta1)
            # theta2_degree = deg2rad(theta2)
            # theta3_degree = deg2rad(theta3)
            # theta4_degree = deg2rad(theta4)

            # print('can ID : {} --> degree : {}'.format(msg.can_id[0], theta1_degree))
            # print('can ID : {} --> degree : {}'.format(msg.can_id[1], theta2_degree))
            # print('can ID : {} --> degree : {}'.format(msg.can_id[3], theta3_degree))
            # print('can ID : {} --> degree : {}'.format(msg.can_id[2], theta4_degree))

            # (4) wheel odometry (unit : m)
            print("x_pos : {} m".format(real_x_pos))
            print("y_pos : {} m".format(real_y_pos))


            ######## etc #########

            # (1) first step
            i += 1

            self.stamp +=1

            # (2) publish data
            wheel_odometry = Float64MultiArray()

            wheel_odometry.data = np.array([real_x_pos, real_y_pos])

            pub.publish(wheel_odometry)

        else :

            ###### rostopic subscibe about each wheel position #######

            theta1 = -(float(msg.accum_deg_now[0]) * 100) / gear_ratio  # radian
            theta2 = float(msg.accum_deg_now[1]) / gear_ratio  # radian
            theta3 = float(msg.accum_deg_now[3]) / gear_ratio  # radian
            theta4 = float(msg.accum_deg_now[2]) / gear_ratio  # radian


            ####### essential data for odometry compuye ########

            # (1) original-update verstion
            # new_odometry_info = self.odometry(theta1, theta2, theta3, theta4, self.odometry_info, dt)

            # real_x_pos = -new_odometry_info[1]
            # real_y_pos = new_odometry_info[2]

            # (2) new-update version
            self.odometry_info = self.odometry(theta1, theta2, theta3, theta4, self.odometry_info, dt)

            real_x_pos = self.odometry_info[1]
            real_y_pos = -self.odometry_info[2]


            ######## append data ########

            # (1) plt data
            self.x_pos_data.append(real_y_pos)
            self.y_pos_data.append(real_x_pos)

            # (2) wheel odometry result data 
            x_plt_data.append(real_y_pos)
            y_plt_data.append(real_x_pos)
            
            # (3) wheel position data

            theta1_pos.append(theta1)
            theta2_pos.append(theta2)
            theta3_pos.append(theta3)
            theta4_pos.append(theta4)

            # (4) All data (stamp, each wheel position, odometry result)

            wheel_position = np.array([self.stamp, theta1, theta2, theta3, theta4, real_y_pos, real_x_pos])
            wheel_data.append(wheel_position)

        
            ####### print data #######

            # (1) stamp
            print("stamp = {}".format(self.stamp))

            # (2) wheel position (unit : radian)
            print('can ID : {} --> radian : {}'.format(msg.can_id[0], theta1))
            print('can ID : {} --> radian : {}'.format(msg.can_id[1], theta2))
            print('can ID : {} --> radian : {}'.format(msg.can_id[3], theta3))
            print('can ID : {} --> radian : {}'.format(msg.can_id[2], theta4))

            # (3) wheel position (unit : degree)

            # theta1_degree = deg2rad(theta1)
            # theta2_degree = deg2rad(theta2)
            # theta3_degree = deg2rad(theta3)
            # theta4_degree = deg2rad(theta4)

            # print('can ID : {} --> degree : {}'.format(msg.can_id[0], theta1_degree))
            # print('can ID : {} --> degree : {}'.format(msg.can_id[1], theta2_degree))
            # print('can ID : {} --> degree : {}'.format(msg.can_id[3], theta3_degree))
            # print('can ID : {} --> degree : {}'.format(msg.can_id[2], theta4_degree))

            # (4) wheel odometry (unit : m)
            print("x_pos : {} m".format(real_x_pos))
            print("y_pos : {} m".format(real_y_pos))

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

    def odometry(self, theta1, theta2, theta3, theta4, past_odometry_info, dt):

        #                   0     1      2       3       4       5       6
        # odometry_info = [phi, x_pos, y_pos, theta1, theta2, theta3, theta4]
        
        ######## kinematic model of mecanum mobile platform ########

        r = 0.0762
        l = 0.23
        w = 0.25225
        alpha = l + w

        F = r/4 * np.array([[-1/alpha, 1/alpha, 1/alpha, -1/alpha], [1, 1, 1, 1], [-1, 1, -1, 1]])
        
        ######## odometry information update about timestep t-1 ########

        phi = past_odometry_info[0]
        x_pos = past_odometry_info[1]
        y_pos = past_odometry_info[2]
        wheel_ang = past_odometry_info[3:7] # [past_theta1, past_theta2, past_theta3, past_theta4]

        ######## calculate delta_wheel_ang about timestep t using wheel_position ########

        recent_theta = np.array([theta1, theta2, theta3, theta4])
        theta_past = np.array([wheel_ang[0], wheel_ang[1], wheel_ang[2], wheel_ang[3]])
        delta_wheel_ang = -(theta_past - recent_theta)
        
        ######## calculate chassis velocity ########

        twist_b = np.dot(F, delta_wheel_ang)
        wbz, vbx, vby = twist_b
        
        ######## {b} coordinate --> {s} coordinate about d_qb ########

        T_sb = np.array([[1.0, 0.0, 0.0],[0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])

        if wbz == 0:
            d_qb = np.array([0, vbx, vby])
        
        else:
            d_qb = np.array([wbz, (vbx * math.sin(wbz) + vby * (math.cos(wbz) - 1))/wbz, (vby * math.sin(wbz) + vbx * (1 - math.cos(wbz)))/wbz])

        d_q = np.dot(T_sb, d_qb)

        ######## odometry information update about timestep t ########

        phi_update = phi + d_q[0]
        x_pos_update = x_pos + d_q[1]
        y_pos_update = y_pos + d_q[2]

        ######## print data #######
        print("phi = {}".format(phi_update))
        print("final_x_pos = {}".format(x_pos_update))
        print("final_y_pos = {}".format(y_pos_update))
        print("----------------------------------------")

        ######## original-result ########
        # new_wheel_ang = wheel_ang + delta_wheel_ang
        # new_odometry_info = np.r_[phi_update, x_pos_update, y_pos_update, new_wheel_ang]

        ######## new-result ########
        new_wheel_ang = np.array([theta1, theta2, theta3, theta4])
        new_odometry_info = np.r_[phi_update, x_pos_update, y_pos_update, new_wheel_ang]

        # return update_odometry_info
        return new_odometry_info


######## init value ########

i = 0

Hz = 250
dt = 1./Hz # degreee difference in 0.02
gear_ratio = 14.0

x_plt_data = []
y_plt_data = []

test_odometry = []

theta1_pos = []
theta2_pos = []
theta3_pos = []
theta4_pos = []

wheel_data = []

####### ROS node ########

rospy.init_node('odometry_msg', anonymous=True)
vis = Visualiser()
sub = rospy.Subscriber("/dev/ttyACM0/sensors/customs", VescGetCustomApp, vis.pose_callback)
pub = rospy.Publisher("/odometry/wheel_velocity", Float64MultiArray, queue_size= 1)

####### show plotting #######
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.legend(loc= 'upper left')
plt.show()

####### save pickle data #######

# (1) wheel odometry result data
with open("/home/jetson/workspace/catkin_ws_localization/src/position_data/pose/x_data_{}.pickle".format(count), "wb") as px:
    pickle.dump(x_plt_data, px)
with open("/home/jetson/workspace/catkin_ws_localization/src/position_data/pose/y_data_{}.pickle".format(count), "wb") as py:
    pickle.dump(y_plt_data, py)

# (2) Each wheel position data
with open("/home/jetson/workspace/catkin_ws_localization/src/position_data/wheel_position/theta1_data_{}.pickle".format(count), "wb") as t1:
    pickle.dump(theta1_pos, t1)
with open("/home/jetson/workspace/catkin_ws_localization/src/position_data/wheel_position/theta2_data_{}.pickle".format(count), "wb") as t2:
    pickle.dump(theta2_pos, t2)
with open("/home/jetson/workspace/catkin_ws_localization/src/position_data/wheel_position/theta3_data_{}.pickle".format(count), "wb") as t3:
    pickle.dump(theta3_pos, t3)
with open("/home/jetson/workspace/catkin_ws_localization/src/position_data/wheel_position/theta4_data_{}.pickle".format(count), "wb") as t4:
    pickle.dump(theta4_pos, t4)

# (3) All data (stamp, each wheel position, odometry result)
with open("/home/jetson/workspace/catkin_ws_localization/src/position_data/all_wheel_position_data_{}.pickle".format(count), "wb") as al:
    pickle.dump(wheel_data, al)
