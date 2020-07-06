#!/usr/env/python
"""
From zed camera topics, create the npy file demo/main.py needs.
data.items() includes 'intrinsics_matrix' 'image' 'depth' 'smoothed_object_pc'
Yongming Qin
2020/05/10 Not finished.
"""

from __future__ import print_function
import rospy
import message_filters
import numpy as np
import ros_numpy


from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

"""
/zedm/zed_node/rgb/camera_info
/zedm/zed_node/rgb/image_rect_color
/zedm/zed_node/depth/depth_registered
/zedm/zed_node/point_cloud/cloud_registered
"""

data = {}

def callback(camerainfo, image, depth, cloud):
    data['intrinsics_matrix'] = np.array(camerainfo.K).reshape(3,3)
    print(data['intrinsics_matrix'])
    data['image'] = ros_numpy.numpify(image)
    print(data['image'].shape)
    data['depth'] = ros_numpy.numpify(depth)
    print(data['depth'].shape)

    pc = ros_numpy.numpify(cloud).flatten()
    ar = np.array(1,3)
    for e in pc:
        if e[0] != np.nan and e[1] != np.nan and e[2] != np.nan:
            t = np.array([[e[0], e[1], e[2]]])
        np.append(ar, t)
    
    print(ar.shape)
    
    data['smoothed_object_pc'] = pc
    print(type(data['smoothed_object_pc']))
    print(data['smoothed_object_pc'])
    print("end of callback")
    rospy.sleep(1)


def listener():
    rospy.init_node('zed_to_npy')
    info_sub = message_filters.Subscriber('/zedm/zed_node/rgb/camera_info', CameraInfo)
    image_sub = message_filters.Subscriber('/zedm/zed_node/rgb/image_rect_color', Image)
    depth_sub = message_filters.Subscriber('/zedm/zed_node/depth/depth_registered', Image)
    cloud_sub = message_filters.Subscriber('/zedm/zed_node/point_cloud/cloud_registered', PointCloud2)

    ats = message_filters.ApproximateTimeSynchronizer([info_sub, image_sub, depth_sub, cloud_sub], slop=0.1, queue_size=10)
    ats.registerCallback(callback)

    rospy.spin()


if __name__ == "__main__":
    listener()