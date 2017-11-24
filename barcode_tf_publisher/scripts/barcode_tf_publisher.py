#!/usr/bin/env python
import rospy
from collections import defaultdict

from geometry_msgs.msg._PoseStamped import PoseStamped
from refills_msgs.msg._Barcode import Barcode
import numpy as np
import tf
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point, do_transform_pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

barcodes_min = defaultdict(lambda: np.ones(3)*np.inf)
barcodes_max = defaultdict(lambda: np.ones(3)*-np.inf)
frame_id = 'map'
tfBuffer = None
tf_listener = None
barcode_pose_sub = None
tf_broadcaster = None
min_scans = 3


def init():
    global tfBuffer, tf_listener, barcode_pose_sub, tf_broadcaster
    rospy.init_node('barcode_tf_publisher')
    tfBuffer = Buffer(rospy.Duration(10))
    tf_listener = TransformListener(tfBuffer)
    tf_broadcaster = tf.TransformBroadcaster()
    rospy.sleep(1)
    barcode_pose_sub = rospy.Subscriber('barcode/pose', Barcode, barcode_sub, queue_size=100)


def transformPose(target_frame, pose):
    global tfBuffer
    transform = tfBuffer.lookup_transform(target_frame,
                                          pose.header.frame_id,  # source frame
                                          pose.header.stamp,  # get the tf at first available time
                                          rospy.Duration(1.0))
    new_pose = do_transform_pose(pose, transform)
    return new_pose

def publish_tf_pose(barcode, barcode_position):
    global frame_id
    tf_broadcaster.sendTransform(barcode_position,
                                 [0,0,0,1],
                                 rospy.Time.now(),
                                 barcode,
                                 frame_id)

def barcode_sub(msg):
    global frame_id, barcodes_min, barcodes_max
    map_pose = transformPose(frame_id, msg.barcode_pose)
    barcodes_min[msg.barcode][0] = min(barcodes_min[msg.barcode][0], map_pose.pose.position.x)
    barcodes_min[msg.barcode][1] = min(barcodes_min[msg.barcode][1], map_pose.pose.position.y)
    barcodes_min[msg.barcode][2] = min(barcodes_min[msg.barcode][2], map_pose.pose.position.z)
    barcodes_max[msg.barcode][0] = max(barcodes_max[msg.barcode][0], map_pose.pose.position.x)
    barcodes_max[msg.barcode][1] = max(barcodes_max[msg.barcode][1], map_pose.pose.position.y)
    barcodes_max[msg.barcode][2] = max(barcodes_max[msg.barcode][2], map_pose.pose.position.z)
    pass


def get_barcode_positions():
    global barcodes_min, barcodes_max
    for k, v in barcodes_min.items():
        mean_pose = (v + barcodes_max[k])/2
        if np.inf not in mean_pose:
            publish_tf_pose(k, mean_pose)


if __name__ == '__main__':
    init()
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        get_barcode_positions()
        r.sleep()
    rospy.spin()
