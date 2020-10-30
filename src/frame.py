#!/usr/bin/env python  
import roslib
import rospy
import tf

from nav_msgs.msg import Odometry


def callback(msg):
    #br = tf.TransformBroadcaster()
    #br.sendTransform((-msg.pose.pose.position.x, 
    #                  msg.pose.pose.position.y,
    #                  -msg.pose.pose.position.z),
    #                 (-msg.pose.pose.orientation.y, 
    #                  msg.pose.pose.orientation.x, 
    #                  msg.pose.pose.orientation.z, 
    #                  msg.pose.pose.orientation.w),
    #                 rospy.Time(),
    #                 "central",
    #                 "map")

    br1 = tf.TransformBroadcaster()
    br1.sendTransform((msg.pose.pose.position.y, 
                       msg.pose.pose.position.x,
                      - msg.pose.pose.position.z),
                     (0, 
                      0, 
                      0, 
                      1),
                     rospy.Time(0,0),
                     "central",
                     "map")

    br2 = tf.TransformBroadcaster()
    br2.sendTransform((0, 
                       0,
                       0),
                      (msg.pose.pose.orientation.x, 
                       msg.pose.pose.orientation.y, 
                       msg.pose.pose.orientation.z, 
                       msg.pose.pose.orientation.w),
                      rospy.Time(0,0),
                      "map",
                      "camera")




if __name__ == '__main__':
    rospy.init_node('tf_camera')
    rospy.Subscriber('odom_t265_sync',
                     Odometry,
                     callback)
    
    rospy.loginfo("TF publisher is run")

    rospy.spin()
