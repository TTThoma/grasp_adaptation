import re

import trimesh
# import collada
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf import transformations

import rospy
import gazebo_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg

# import baxter_core_msgs.srv
# import baxter_interface
# from cv_bridge import CvBridge

def spawn_gazebo_model(spawn_client, model_name="basket", object_name="start_box", x=0.6725, y=0.0, z=0.7825, qx=0, qy=0, qz=0, qw=1):
    try:
        model_sdf = rospy.get_param("/"+model_name+"_description")
        model_sdf = re.sub(r"/gazebo/"+model_name+"/contact*", "/gazebo/"+object_name+"/contact", model_sdf)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        resp_sdf = spawn_client(object_name, model_sdf, "/", pose, "world")
        rospy.sleep(0.01)
        return pose
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
        return None


def set_object_state(set_model,object_name,h=0.9):
    rnd_x = 0.025*(2.*np.random.uniform()-1.) + 0.6
    rnd_y = 0.025*(2.*np.random.uniform()-1.) + 0.3
    rnd_z = h
    rnd_roll = 2.*np.pi*np.random.uniform()
    rnd_pitch = 2.*np.pi*np.random.uniform()
    rnd_ywa = 2.*np.pi*np.random.uniform()
    rnd_quaternion = transformations.quaternion_from_euler(rnd_roll,rnd_pitch,rnd_ywa)

    pose = geometry_msgs.msg.Pose()
    pose.orientation.x = rnd_quaternion[0]
    pose.orientation.y = rnd_quaternion[1]
    pose.orientation.z = rnd_quaternion[2]
    pose.orientation.w = rnd_quaternion[3]

    pose.position.x = rnd_x
    pose.position.y = rnd_y
    pose.position.z = rnd_z

    velocity = geometry_msgs.msg.Twist()
    velocity.linear.x = 0.0
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0

    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = 0.0

    set_model_state_req = gazebo_msgs.srv.SetModelStateRequest()
    set_model_state_req.model_state.model_name = object_name
    set_model_state_req.model_state.pose = pose
    set_model_state_req.model_state.twist = velocity
    set_model_state_req.model_state.reference_frame = "world"

    set_model_state_resp = set_model(set_model_state_req)
    rospy.sleep(0.01)


def reset_gazebo_model(spawn_client, object_name, pose):
    try:
        model_sdf = rospy.get_param("/"+model_name+"_description")
        model_sdf = re.sub(r"/gazebo/"+model_name+"/contact*", "/gazebo/"+object_name+"/contact", model_sdf)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        resp_sdf = spawn_client(object_name, model_sdf, "/", pose, "world")
        rospy.sleep(0.01)
        return pose
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
        return None


def delete_gazebo_model(delete_model, object_name="start_box"):
    try:
        resp_sdf = delete_model(object_name)
        rospy.sleep(0.01)
    except rospy.ServiceException, e:
        rospy.logerr("Delete Model service call failed: {0}".format(e))


def get_gazebo_model_state(get_model_state, object_name="object0"):
    try:
        model_state_resp = get_model_state(object_name,"world")
    except rospy.ServiceException, e:
        rospy.logerr("Get Model State service call failed: {0}".format(e))
    return model_state_resp.pose
