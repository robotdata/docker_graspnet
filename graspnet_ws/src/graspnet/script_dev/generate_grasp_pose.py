#!/usr/bin/env python3
"""
Subscribe to a PointCloud2 topic named 'object_pc'.
This is the object information.
Then the GraspNet functions are called to generate the grasp poses.
The first pose is published to a Pose topic named generated_grasp_pose.
Yongming Qin
2020/07/03
"""

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

import os
import sys
os.chdir("/graspnet_ws/src/graspnet/pytorch_6dof-graspnet")
# print(os.listdir())
sys.path.append("/graspnet_ws/src/graspnet/pytorch_6dof-graspnet")

import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import glob
import mayavi.mlab as mlab

# From GraspNet
import grasp_estimator
from utils.visualization_utils import *
from utils import utils
from data import DataLoader

# parameters
object_pc = None # Pointcloud of the object to be grasped
grasp_pose = Pose()

# parameters
class Args(object):
    def __init__(self):
        pass
args = Args()
# args.grasp_sampler_folder = 'checkpoints/gan_pretrained/'
args.grasp_sampler_folder = 'checkpoints/vae_pretrained'
args.grasp_evaluator_folder = 'checkpoints/evaluator_pretrained/'
# args.refinement_method = 'sampling'
args.refinement_method = 'gradient'
args.refine_steps = 10
args.npy_folder = 'demo/data/'
args.threshold = 0.8
args.choose_fn = 'better_than_threshold'
args.target_pc_size = 1024
args.num_grasp_samples = 10
args.generate_dense_grasps = False
args.batch_size = 30
args.train_data = False
if args.train_data:
    args.dataset_root_folder = ''


# GraspNet
grasp_sampler_args = utils.read_checkpoint_args(args.grasp_sampler_folder)
grasp_sampler_args.is_train = False
#print(grasp_sampler_args)
grasp_evaluator_args = utils.read_checkpoint_args(args.grasp_evaluator_folder)
grasp_evaluator_args.continue_train = True
#print(grasp_evaluator_args)
estimator = grasp_estimator.GraspEstimator(grasp_sampler_args, grasp_evaluator_args, args)


def callback(msg):
    object_pc = ros_numpy.numpify(msg)
    generated_grasps, generated_scores = estimator.generate_and_refine_grasps(object_pc) # two lists
    if len(generated_grasps) > 0:
        grasp_pose.position.x = generated_grasps[0][0,3]
        grasp_pose.position.y = generated_grasps[0][1,3]
        grasp_pose.position.z = generated_grasps[0][2,3]
        rotation_matrix = [[generated_grasps[0][0,0], generated_grasps[0][0,1], generated_grasps[0][0,2]],
                           [generated_grasps[0][1,0], generated_grasps[0][1,1], generated_grasps[0][1,2]],
                           [generated_grasps[0][2,0], generated_grasps[0][2,1], generated_grasps[0][2,2]]]
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()
        grasp_pose.prientation.x = quat[0]
        grasp_pose.prientation.y = quat[1]
        grasp_pose.prientation.z = quat[2]
        grasp_pose.prientation.w = quat[3]
        pub.publish(grasp_pose)



if __name__ == '__main__':
    rospy.init_node('grasp_generator', anonymous=True)
    pub = rospy.Publisher('generated_grasp_pose', Pose, queue_size=1)
    rospy.Subscriber('object_pc', PointCloud2, callback)
    while not rospy.is_shutdown():
        print("Subscribing...")
        rospy.sleep(2.)
