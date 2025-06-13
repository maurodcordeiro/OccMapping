#!/usr/bin/env python3
import rospy as rp
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
import tf
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


# Odom -> Base_Link
def odom_callback(msg: Odometry):
    m = TransformStamped()
    m.header.stamp = msg.header.stamp
    m.header.frame_id = "odom"
    m.child_frame_id = "base_link"
    m.transform.translation.x = msg.pose.pose.position.x
    m.transform.translation.y = msg.pose.pose.position.y
    m.transform.translation.z = msg.pose.pose.position.z
    m.transform.rotation = msg.pose.pose.orientation
    pub_odom_link.sendTransform(m)

    return


def static_frame(frame_id: str, child_frame_id: str):
    m = TransformStamped()
    m.header.stamp = rp.Time.now()
    m.header.frame_id = frame_id
    m.child_frame_id = child_frame_id
    m.transform.translation.x = 0
    m.transform.translation.y = 0
    m.transform.translation.z = 0
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    m.transform.rotation.x = quaternion[0]
    m.transform.rotation.y = quaternion[1]
    m.transform.rotation.z = quaternion[2]
    m.transform.rotation.w = quaternion[3]

    return m

if __name__ == '__main__':
    global pub

    rp.init_node('odom_tf_broadcaster')

    pub = tf.TransformBroadcaster(queue_size=10)

    # Statics
    pub_link_scan = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    m = static_frame("base_link", "base_scan")
    pub_link_scan.sendTransform(m)

    # Normal
    global pub_odom_link
    pub_odom_link = tf2_ros.TransformBroadcaster()
    sub = rp.Subscriber("/odom", Odometry, odom_callback)


    rp.spin()

