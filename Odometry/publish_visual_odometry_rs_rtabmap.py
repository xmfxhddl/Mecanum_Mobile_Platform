#! /usr/bin/env python

import rospy
import numpy as np
import pickle
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from rtabmap_ros.msg import MapGraph
from geometry_msgs.msg import Pose2D, Transform
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32

#######################################
count = "11M_24D_025"
#######################################

class Visualiser():

    global x_plt_data
    global y_plt_data

    stamp = 0

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'r', linewidth=2, label="visual_inertial_odometry")
        
        self.x_data, self.y_data = [], []

    def plot_init(self):
        # self.ax.set_xlim(-5, 5)
        # self.ax.set_ylim(-5, 5)

        self.ax.set_aspect('equal', adjustable = 'box')

        self.ax.set_xticks([-4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0]) # plt range x value
        self.ax.set_yticks([-1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4.0]) # plt range y value

        return self.ln

    def pose_callback(self, msg):
        
        ####### rs_rtabmap -> rostopic /t265/odom/sample
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        ####### rostopic orientation (type : quarternion) #######
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q4 = msg.pose.pose.orientation.w

        ####### plt data add #######
        self.x_data.append(-y)
        self.y_data.append(x)

        ####### save data add #######
        x_plt_data.append(-y)
        y_plt_data.append(x)

        ###### quarternion data add #######
        q1_data.append(q1)
        q2_data.append(q2)
        q3_data.append(q3)
        q4_data.append(q4)

        ####### quarternion ----> radian #######
        t0 = 2.0 * (q4 * q1 + q2 * q3)
        t1 = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll_x = math.atan2(t0,t1)

        t2 = 2.0 * (q4 * q2 - q3 * q1)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = 2.0 * (q4 * q3 + q1 *q2)
        t4 = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw_z = math.atan2(t3, t4)

        ####### radian data add #######
        radian_data.append([yaw_z])

        degree = yaw_z * (180 / math.pi)

        xyd_pose = np.array([int(self.stamp), -y, x, yaw_z])

        xyd_data.append(xyd_pose)

        ####### print data #######
        print(self.stamp)
        print("x pose : {}".format(-y))
        print("y pose : {}".format(x))
        print("radian : {}".format(yaw_z))

        self.stamp +=1

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln

    def path_count_callback(self, msg):
        path=msg.data
        # print(path)
        

x_plt_data = []
y_plt_data = []

pose_data = []

q1_data = []
q2_data = []
q3_data = []
q4_data = []

radian_data = []

xyd_data = []

rospy.init_node("visual_inertial_odometry", anonymous=True)
vis = Visualiser()
# rviz path information (x,y)
# sub = rospy.Subscriber("/rtabmap/mapPath", Path, vis.pose_callback)

# T265 tracking camera information (x, y)
sub = rospy.Subscriber("/t265/odom/sample", Odometry, vis.pose_callback)
sub1 = rospy.Subscriber("/odometry/path_count", Int32, vis.path_count_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.legend(loc="upper left")
plt.show()


# pickle : x, y ---> pose
with open("/home/jetson/workspace/catkin_ws_visual_localization/src/data/pose/x_data_visual_{}.pickle".format(count), "wb") as fx:
    pickle.dump(x_plt_data, fx)
with open("/home/jetson/workspace/catkin_ws_visual_localization/src/data/pose/y_data_visual_{}.pickle".format(count), "wb") as fy:
    pickle.dump(y_plt_data, fy)

#pickle : q1, q2, q3, q4 ---> quaternion
with open("/home/jetson/workspace/catkin_ws_visual_localization/src/data/quaternion/q1_data_{}.pickle".format(count), "wb") as fq1:
    pickle.dump(q1_data, fq1)
with open("/home/jetson/workspace/catkin_ws_visual_localization/src/data/quaternion/q2_data_{}.pickle".format(count), "wb") as fq2:
    pickle.dump(q2_data, fq2)
with open("/home/jetson/workspace/catkin_ws_visual_localization/src/data/quaternion/q3_data_{}.pickle".format(count), "wb") as fq3:
    pickle.dump(q3_data, fq3)
with open("/home/jetson/workspace/catkin_ws_visual_localization/src/data/quaternion/q4_data_{}.pickle".format(count), "wb") as fq4:
    pickle.dump(q4_data, fq4)

# pickle : stamp, x, y, radian
with open("/home/jetson/workspace/catkin_ws_visual_localization/src/data/xyd_pose_data_{}.pickle".format(count), "wb") as xyd:
    pickle.dump(xyd_data, xyd)
