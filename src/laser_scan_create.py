#!/usr/bin/python3.6
import rospy
import cv2
import numpy as np
import math

# for LaserScan
#from sensor_msgs import laser_scan
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

def main():

    # node init
    rospy.init_node('rover_laser_scan_Generator', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    pub = rospy.Publisher('scan_generated', LaserScan, queue_size=2)

    no_points = 300

    scan = LaserScan()
    scan.header.frame_id = "map"
    scan.angle_min = -math.pi+(40*math.pi/180)
    scan.angle_max = math.pi-(15*math.pi/180)
    scan.angle_increment = 2*math.pi/no_points
    scan.time_increment = (1 / 30) / no_points;
    scan.range_min = 0.0
    scan.range_max = 100.0
    #scan.set_ranges_size(100)
    #scan.set_intensities_size(100)

    for i in range(1, no_points):
        if i < (no_points/2):
            if i/(no_points/2)*math.pi < abs(scan.angle_min):
                scan.ranges.append(10+ i%10 * 0.1)
                scan.intensities.append(no_points)
        elif i > (no_points/2):
            if i/no_points*math.pi < abs(scan.angle_max):
                scan.ranges.append(10- i%10 * 0.1)
                scan.intensities.append(no_points)

    print("laserscan generated")
    # main loop
    while not rospy.is_shutdown():
        # get and publish image and camera_info
        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    main()
