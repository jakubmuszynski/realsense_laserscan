#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d
import message_filters
import math as m

# for trajectory
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from trajectory_fun import get_path_position_orientation
from nav_msgs.msg import Odometry

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header

def cameras_callback(odom_msg, image_msg):
    timestamp_pc = image_msg.header.stamp
    timestamp_tr = odom_msg.header.stamp
    #print("IM: ", timestamp_pc, " TR: ", timestamp_tr, " difference: ", timestamp_tr - timestamp_pc)

    image_msg.header.stamp = rospy.Time()
    odom_msg.header.stamp = rospy.Time()

    pub_odom.publish(odom_msg)
    pub_align_depth.publish(image_msg)

# Node init
rospy.init_node('sync', anonymous = True)

# Subscriber definition
odometry = message_filters.Subscriber('odom_t265', Odometry)
point_cloud = message_filters.Subscriber('align_depth', Image)
ts = message_filters.ApproximateTimeSynchronizer([odometry, point_cloud], 2, 0.00000005, allow_headerless=True)
ts.registerCallback(cameras_callback)

# Publisher definition
pub_odom = rospy.Publisher('odom_t265_sync', Odometry, queue_size=20)
pub_align_depth = rospy.Publisher("align_depth_sync", Image, queue_size=2)
rate = rospy.Rate(30) # 30hz

#print("Start node")
rospy.loginfo("Realsense server is run")


while not rospy.is_shutdown():

    # Get data from cameras
    rate.sleep()
