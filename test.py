# from binpicking_environment import *
#
# a = BinPickingEnv()


import struct
from copy import deepcopy

import rospy
import gazebo_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
# import baxter_core_msgs.srv

import tf.transformations
# import baxter_interface

import numpy as np
from utils import *
from baxter_pickplace_interface import *
from matplotlib import pyplot as plt

class BinPickingEnv():
    def __init__(self,):
        rospy.init_node("ik_pick_and_place_demo")

        # limb = "left"
        # hover_distance = 0.12
        # pnp = PickAndPlace(limb, hover_distance)
        # pnp.move_to_start()

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', gazebo_msgs.srv.SpawnModel)
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', gazebo_msgs.srv.SpawnModel)

        camera_quaternion = tf.transformations.quaternion_from_euler(0., np.pi / 2., 0.)
        spawn_gazebo_model(spawn_urdf, "realsense", "rs200", 0.65, 0.0, 1.7825,
                           qx=camera_quaternion[0], qy=camera_quaternion[1], qz=camera_quaternion[2],
                           qw=camera_quaternion[3])
        spawn_gazebo_model(spawn_sdf, "cafe_table", "cafe_table", 0.8, 0.0, 0.0)
        spawn_gazebo_model(spawn_sdf, "basket", "start_box", 0.65, 0.0, 0.7825)

        object_pose_list = []
        number_of_object = 1
        for i in range(number_of_object):
            rnd_x = 0.02 * (2. * np.random.uniform() - 1.) + 0.65
            rnd_y = 0.02 * (2. * np.random.uniform() - 1.)
            rnd_z = 1.0
            rnd_roll = 2. * np.pi * np.random.uniform()
            rnd_pitch = 2. * np.pi * np.random.uniform()
            rnd_ywa = 2. * np.pi * np.random.uniform()
            rnd_quaternion = tf.transformations.quaternion_from_euler(rnd_roll, rnd_pitch, rnd_ywa)
            object_pose = spawn_gazebo_model(spawn_sdf, "t_shape", "object" + str(i),
                                             rnd_x, rnd_y, rnd_z,
                                             rnd_quaternion[0], rnd_quaternion[1], rnd_quaternion[2], rnd_quaternion[3])
            object_pose_list.append(object_pose)

        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", gazebo_msgs.srv.GetModelState)
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", gazebo_msgs.srv.SetModelState)

        highest_object_pose = None
        highest_object_name = None
        for i in range(number_of_object):
            object_pose = get_gazebo_model_state(get_model_state, "object" + str(i))
            if highest_object_pose is None:
                highest_object_pose = object_pose
                highest_object_name = "object" + str(i)
            elif (highest_object_pose.position.x - 0.65) ** 2 + highest_object_pose.position.y ** 2 < (object_pose.position.x - 0.65) ** 2 + object_pose.position.y ** 2:
                highest_object_pose = object_pose
                highest_object_name = "object" + str(i)

        # file_name = "/home/dof6/.gazebo/models/t_shape/meshes/t_shape.dae"
        # trimesh_data, _ = load_dae_to_trimesh(file_name)
        # gripper_contact_list, gripper_quaternion_list, gripper_width_list, quality_list, extra_information_list = sample_antipodal_grasp(
        #     trimesh_data, number_of_contact=100, number_of_quaternion=37)
a = BinPickingEnv()
