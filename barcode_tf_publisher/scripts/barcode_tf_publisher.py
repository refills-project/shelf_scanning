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

barcodes = defaultdict(list)
barcode_positions = {}
frame_id = 'map'
tfBuffer = None
tf_listener = None
barcode_pose_sub = None
tf_broadcaster = None
min_scans = 10


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
    global frame_id, barcodes
    map_pose = transformPose(frame_id, msg.barcode_pose)
    barcodes[msg.barcode].append(map_pose.pose.position)


def get_barcode_positions():
    global barcodes, barcode_positions
    for k, v in barcodes.items():
        points = []
        for barcode_position in v:
            points.append([barcode_position.x,
                           barcode_position.y,
                           barcode_position.z, ])
        mean_pose = (np.min(points, axis=0) + np.max(points, axis=0))/2
        if len(points) > 10:
            print(k, mean_pose)
            publish_tf_pose(k, mean_pose)


if __name__ == '__main__':
    init()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        get_barcode_positions()
        rospy.sleep(.5)
    rospy.spin()
