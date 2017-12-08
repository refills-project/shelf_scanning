#!/usr/bin/env python
import rospy
from collections import defaultdict

from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Vector3 import Vector3
from refills_msgs.msg._Barcode import Barcode
import numpy as np
import tf
from std_msgs.msg._ColorRGBA import ColorRGBA
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point, do_transform_pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg._Marker import Marker

barcodes_min = defaultdict(lambda: np.ones(3)*np.inf)
barcodes_max = defaultdict(lambda: np.ones(3)*-np.inf)
frame_id = 'map'
tfBuffer = None
tf_listener = None
barcode_pose_sub = None
tf_broadcaster = None
marker_pub = None
min_scans = 3

refills_models_path = 'package://refills_models/'

barcode_to_object = {
    #shelf1
    #row4
    '22005632': 'Shelf1/CalgonitFinishVorratspack/SM_CalgonitFinishVorratspack_reexport.dae',
    '24476003': 'Shelf1/CalgonitFinishVorratspack/Axle_Full.dae',
    '25011555': '/home/ichumuh/homer_ws/src/iai_maps/iai_shelfs/meshes/Shelf_3.dae',
    '25011562': 'finish small pack',
    '25349108': 'finish small pack',
    #row3
    '21240362': 'finish small pack',
    '21766718': 'finish small pack',
    '24227711': 'finish small pack',
    '23313729': 'finish small pack',
    '23779549': 'finish small pack',
    '21923890': 'finish small pack',
    '22270306': 'finish small pack',
    #row2
    '20047283': 'finish small pack',
    '21960680': 'finish small pack',
    '23323841': 'finish small pack',
    '25231298': 'finish small pack',
    '24249294': 'finish small pack',
    '22355423': 'finish small pack',
    #row1
    '25001839': 'finish small pack',
    '20460884': 'finish small pack',
    '22622891': 'finish small pack',
    '20100551': 'finish small pack',
    '20156527': 'finish small pack',
    '25169379': 'finish small pack',
    '21254819': 'finish small pack',
    #row0
    '20279950': 'finish small pack',
    '25442052': 'finish small pack',
    '23841604': 'finish small pack',
    '24573191': 'finish small pack',
    '25348125': 'finish small pack',
    '24026109': 'finish small pack',
    '24339612': 'finish small pack',
    '25079234': 'finish small pack',
    #shelf2
    #row0
    '20134624': 'Shelf2/',
    '24588232': 'Shelf2/',
    '25222739': 'Shelf2/',
    '21026966': 'Shelf2/',
    '21026942': 'Shelf2/',
    '25191516': 'Shelf2/',
    #row1
    '25229905': 'Shelf2/',
    '25229912': 'Shelf2/',
    '25230093': 'Shelf2/',
    '25229851': 'Shelf2/',
    '25230109': 'Shelf2/',
    #row2
    '25229899': 'Shelf2/',
    '25229882': 'Shelf2/',
    '22947635': 'Shelf2/',
    '22947604': 'Shelf2/',
    '24491815': 'Shelf2/',
    '24491822': 'Shelf2/',
    '25072372': 'Shelf2/',
    '20369422': 'Shelf2/',
    '20369460': 'Shelf2/',
    '20349453': 'Shelf2/',
    '20369880': 'Shelf2/',
    '21907555': 'Shelf2/',
    #row3
    '21557774': 'Shelf2/',
    '24639408': 'Shelf2/',
    '25393552': 'Shelf2/',
    #Shelf3
    #row0
    '22607256': 'Shelf3/',
    '22607072': 'Shelf3/',
    '25543018': 'Shelf3/',
    '24044912': 'Shelf3/',
    '25218732': 'Shelf3/',
    '24025898': 'Shelf3/',
    '22606679': 'Shelf3/',
    #row1
    '25206982': 'Shelf3/',
    '25206968': 'Shelf3/',
    '25206890': 'Shelf3/',
    '25206883': 'Shelf3/',
    '24235761': 'Shelf3/',
    '24220682': 'Shelf3/',
    '25118605': 'Shelf3/',
    '22607003': 'Shelf3/',
    '22606969': 'Shelf3/',
    '23995994': 'Shelf3/',
    #row2
    '25354755': 'Shelf3/',
    '25197631': 'Shelf3/',
    '25202090': 'Shelf3/',
    '25202120': 'Shelf3/',
    '25238280': 'Shelf3/',
    '25238297': 'Shelf3/',
    '25238235': 'Shelf3/',
    '25238303': 'Shelf3/',
    '24727877': 'Shelf3/',
    '22938756': 'Shelf3/',
    '25369793': 'Shelf3/',
    '24115520': 'Shelf3/',
    #row3
    '24604703': 'Shelf3/',
    '24602013': 'Shelf3/',
    '25369526': 'Shelf3/',
    '25369434': 'Shelf3/',
    '25369700': 'Shelf3/',
    '25653298': 'Shelf3/',
    '25653359': 'Shelf3/',
    '25188752': 'Shelf3/',
    '25367607': 'Shelf3/',
    '25348031': 'Shelf3/',
    '25188769': 'Shelf3/',
    '25443752': 'Shelf3/',
    '25443820': 'Shelf3/',
    '25537338': 'Shelf3/',
    #row4
    '25447606': 'Shelf3/',
    '25447906': 'Shelf3/',
    '25447880': 'Shelf3/',
    '25662177': 'Shelf3/',
    '25447828': 'Shelf3/',
    '25120189': 'Shelf3/',
    '25120363': 'Shelf3/',
    '25537352': 'Shelf3/',
    '25537369': 'Shelf3/',
    '25537260': 'Shelf3/',
    '25537154': 'Shelf3/',

}

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

def publi7sh_marker(ean):
    global marker_pub, barcode_to_object
    if marker_pub is None:
        marker_pub = rospy.Publisher('shelf_objects', Marker, queue_size=100)
    m = Marker()
    text = Marker()
    text.header.frame_id = ean
    m.header.frame_id = ean
    m.pose.orientation.w = 1
    m.action = Marker.ADD
    text.action = Marker.ADD
    text.type = Marker.TEXT_VIEW_FACING
    text.text = ean
    text.scale = Vector3(0,0,.05)
    text.color = ColorRGBA(1,1,1,1)
    text.pose.position.z = 0.05
    if ean in barcode_to_object:
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = barcode_to_object[ean]
        m.scale = Vector3(1,1,1)
        m.color = ColorRGBA(0,0,0,0)
        m.mesh_use_embedded_materials = True
    else:
        m.type = Marker.CUBE
        m.scale = Vector3(.05,.05,.05)
        m.color = ColorRGBA(.5,.5,.5,1.0)

    m.text = ean
    m.ns = ean
    text.ns = ean
    m.id = 2
    marker_pub.publish(m)
    marker_pub.publish(text)

def get_barcode_positions():
    global barcodes_min, barcodes_max
    for k, v in barcodes_min.items():
        mean_pose = (v + barcodes_max[k])/2
        if np.inf not in mean_pose:
            publish_tf_pose(k, mean_pose)
            publish_marker(k)


if __name__ == '__main__':
    init()
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        get_barcode_positions()
        r.sleep()
    rospy.spin()
