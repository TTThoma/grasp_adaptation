import re

import trimesh
import collada
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf import transformations

import rospy
import gazebo_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg

import baxter_core_msgs.srv
import baxter_interface
from cv_bridge import CvBridge


def load_dae_to_trimesh(file_name):
    mesh_data = collada.Collada(filename=file_name)
    unitmeter = mesh_data.assetInfo.unitmeter

    vertex_index_start = 0
    vertices = []
    faces = []

    for geometry in mesh_data.geometries:
        for primitive in geometry.primitives:
            if not isinstance(primitive,collada.triangleset.TriangleSet):
                triangelset = primitive.triangleset()
            else:
                triangelset = primitive
            vertices.append(triangelset.vertex * unitmeter)
            faces.append(triangelset.vertex_index + vertex_index_start)
            vertex_index_start += vertices[-1].shape[0]

    vertices = np.concatenate(vertices,axis=0)
    faces = np.concatenate(faces,axis=0)

    trimesh_data = trimesh.Trimesh(vertices=vertices,faces=faces)
    return trimesh_data, mesh_data


def visualize_trimesh(trimesh_data):
    ax = plt.gca(projection='3d')
    ax.plot_trisurf(trimesh_data.vertices[:, 0], trimesh_data.vertices[:, 1], trimesh_data.vertices[:, 2], triangles=trimesh_data.faces,
                    linewidth=0.2, antialiased=True, color=[0., 0., 1., 0.2], edgecolor='gray')
    return ax


def sample_antipodal_grasp(trimesh_data, number_of_contact=100, number_of_quaternion=5, gripper_width_max=0.08, gripper_depth_max=0.15, mu=0.1):
    contact_r_points, contact_r_faces_indices = trimesh_data.sample(number_of_contact,return_index=True)
    contact_r_face_normals = trimesh_data.face_normals[contact_r_faces_indices,:]
    
    trimesh_data_ray = trimesh.ray.ray_triangle.RayMeshIntersector(trimesh_data)
    ray_info = trimesh_data_ray.intersects_id(contact_r_points-0.005*contact_r_face_normals, -contact_r_face_normals, return_locations=True, multiple_hits=False)
    ray_indices = ray_info[1]

    contact_r_points = contact_r_points[ray_indices,:]
    contact_r_faces_indices = contact_r_faces_indices[ray_indices]
    contact_r_face_normals = contact_r_face_normals[ray_indices,:]

    contact_l_points = ray_info[2]
    contact_l_faces_indices = ray_info[0]
    contact_l_face_normals = trimesh_data.face_normals[contact_l_faces_indices,:]
    
    gripper_axes = contact_l_points - contact_r_points
    gripper_widths = np.sqrt(np.sum(gripper_axes**2,axis=1,keepdims=True))
    gripper_axes = gripper_axes/gripper_widths
    contact_l_points += 0.001*gripper_axes
    contact_r_points -= 0.001*gripper_axes
    
    gripper_contact_list = []
    gripper_quaternion_list = []
    gripper_width_list = []
    extra_information_list = []
    quality_list = []
    
    for gripper_width, gripper_axis, contact_l_point, contact_l_face_normal, contact_r_point, contact_r_face_normal in zip(gripper_widths, gripper_axes, contact_l_points, contact_l_face_normals, contact_r_points, contact_r_face_normals):
        contact_c_point = (contact_r_point + contact_l_point)/2.
        
        rotation_quat = transformations.quaternion_about_axis(2*np.pi*np.random.uniform(), np.random.uniform([3,]))
        approaching_axis0 = np.cross(gripper_axis, np.matmul(transformations.quaternion_matrix(rotation_quat)[:3,:3], gripper_axis))
        for i in range(number_of_quaternion):
            rotation_quat = transformations.quaternion_about_axis(2*np.pi*i/(number_of_quaternion-1), gripper_axis)
            approaching_axis = np.matmul(transformations.quaternion_matrix(rotation_quat)[:3,:3],approaching_axis0)
            approaching_axis = approaching_axis/np.sqrt(np.sum(approaching_axis**2))

            l_gripper_points = []
            r_gripper_points = []
            for k in range(10):
                r_gripper_points.append(contact_r_point - k/9.*gripper_depth_max*approaching_axis)
            for k in reversed(range(10)):
                l_gripper_points.append(contact_l_point - k/9.*gripper_depth_max*approaching_axis)
            gripper_bar_points = [(r_gripper_points[-1]+l_gripper_points[0])/2.]
            all_gripper_points = r_gripper_points + gripper_bar_points + l_gripper_points 
            all_gripper_points = np.asarray(all_gripper_points)
            sdf_gripper_points = trimesh.proximity.signed_distance(trimesh_data, all_gripper_points)

            quality = 1.0
            if gripper_width > gripper_width_max: 
                quality = 0.0
            if np.sqrt(1./(1.+mu**2)) > np.abs(np.sum(gripper_axis*contact_r_face_normal)) and np.sqrt(1./(1.+mu**2)) > np.abs(np.sum(gripper_axis*contact_l_face_normal)):
                quality = 0.0
            if (sdf_gripper_points > 0.).any():
                quality = 0.0

            x_axis = np.cross(gripper_axis, approaching_axis) 
            y_axis = gripper_axis
            z_axis = approaching_axis

            gripper_rotation_mtx = np.identity(4)
            gripper_rotation_mtx[:3,0] = x_axis
            gripper_rotation_mtx[:3,1] = y_axis
            gripper_rotation_mtx[:3,2] = z_axis
            gripper_quaternion = transformations.quaternion_from_matrix(gripper_rotation_mtx)
            
            extra_information = {
                'contact_r_point' : contact_r_point, 
                'contact_l_point' : contact_l_point, 
                'all_gripper_points' : all_gripper_points,
                'approaching_axis' : approaching_axis,
                'gripper_axis' : gripper_axis
            }
            
            gripper_contact_list.append(contact_c_point)
            gripper_quaternion_list.append(gripper_quaternion)
            gripper_width_list.append(gripper_width)
            extra_information_list.append(extra_information)
            quality_list.append(quality)
            
            for j in range(2):
                print(j)
                rot_quaternion = transformations.quaternion_about_axis(np.pi*(j-1)/2.,approaching_axis)
                contact_c_point = (contact_r_point+contact_l_point)/2.
                contact_r_point = rotate_quaternion_vector(rot_quaternion, contact_r_point-contact_c_point) + contact_c_point
                contact_l_point = rotate_quaternion_vector(rot_quaternion, contact_l_point-contact_c_point) + contact_c_point

                closest_points, _, triangle_id = trimesh.proximity.closest_point(trimesh_data,[contact_r_point, contact_l_point])
                contact_r_point = closest_points[0]
                contact_l_point = closest_points[1]

                contact_r_face_normal = trimesh_data.face_normals[triangle_id[0],:]
                contact_l_face_normal = trimesh_data.face_normals[triangle_id[1],:]

                gripper_axis = contact_l_point - contact_r_point
                gripper_width = np.sqrt(np.sum(gripper_axis**2))
                gripper_axis = gripper_axis/gripper_width

                l_gripper_points = []
                r_gripper_points = []
                for k in range(10):
                    r_gripper_points.append(contact_r_point - k/9.*gripper_depth_max*approaching_axis)
                for k in reversed(range(10)):
                    l_gripper_points.append(contact_l_point - k/9.*gripper_depth_max*approaching_axis)
                gripper_bar_points = [(r_gripper_points[-1]+l_gripper_points[0])/2.]
                all_gripper_points = r_gripper_points + gripper_bar_points + l_gripper_points 
                all_gripper_points = np.asarray(all_gripper_points)
                sdf_gripper_points = trimesh.proximity.signed_distance(trimesh_data, all_gripper_points)

                quality = 1.0
                if (sdf_rot_gripper_points > 0.).any():
                    quality = 0.0
                if np.sqrt(1./(1.+mu**2)) > np.abs(np.sum(gripper_axis*contact_r_face_normal)) and np.sqrt(1./(1.+mu**2)) > np.abs(np.sum(gripper_axis*contact_l_face_normal)):
                    quality = 0.0
                if (sdf_gripper_points > 0.).any():
                    quality = 0.0

                x_axis = np.cross(gripper_axis, approaching_axis) 
                y_axis = gripper_axis
                z_axis = approaching_axis

                gripper_rotation_mtx = np.identity(4)
                gripper_rotation_mtx[:3,0] = x_axis
                gripper_rotation_mtx[:3,1] = y_axis
                gripper_rotation_mtx[:3,2] = z_axis
                gripper_quaternion = transformations.quaternion_from_matrix(gripper_rotation_mtx)

                extra_information = {
                    'contact_r_point' : contact_r_point, 
                    'contact_l_point' : contact_l_point, 
                    'all_gripper_points' : all_gripper_points,
                    'approaching_axis' : approaching_axis,
                    'gripper_axis' : gripper_axis
                }

                gripper_contact_list.append(contact_c_point)
                gripper_quaternion_list.append(gripper_quaternion)
                gripper_width_list.append(gripper_width)
                extra_information_list.append(extra_information)
                quality_list.append(quality)
            
    return gripper_contact_list, gripper_quaternion_list, gripper_width_list, quality_list, extra_information_list


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


def quaternion2list(quaternion):
    quaternion_np = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return quaternion_np


def rotate_quaternion_vector(orientation, vector):
    return np.matmul(transformations.quaternion_matrix(quaternion2list(orientation))[:3,:3],vector)


def position2list(position):
    return [position.x, position.y, position.z]


def get_grasp_result(object_name="object0"):
    for _ in range(3):
        contact_information = rospy.wait_for_message("/gazebo/"+object_name+"/contact", gazebo_msgs.msg.ContactsState)
    if len(contact_information.states) > 0:
        grasp_result1 = False
        grasp_result2 = False

        for state in contact_information.states:
            if "l_gripper_r_finger" in state.collision2_name:
                grasp_result1 = True
            if "l_gripper_l_finger" in state.collision2_name:
                grasp_result2 = True

        if grasp_result1 and grasp_result2:
            grasp_result = True
            rospy.loginfo(str(contact_information.header.seq)+": Contact with Gripper")
        else:
            grasp_result = False
            rospy.loginfo(str(contact_information.header.seq)+": No Contact with Gripper")
    else:
        grasp_result = False
        rospy.loginfo(str(contact_information.header.seq)+": No Contact")
    return grasp_result


def get_rgbd_image():
    bridge = CvBridge()
    
    color_camera_info = rospy.wait_for_message("/r200/camera/color/camera_info", sensor_msgs.msg.CameraInfo)
    depth_camera_info = rospy.wait_for_message("/r200/camera/depth/camera_info", sensor_msgs.msg.CameraInfo)
    
    depth_img = rospy.wait_for_message("/r200/camera/depth/image_raw", sensor_msgs.msg.Image)
    depth_img.encoding = "mono16"
    color_img = rospy.wait_for_message("/r200/camera/color/image_raw", sensor_msgs.msg.Image)
    
    observation = {}
    observation["depth"] = bridge.imgmsg_to_cv2(depth_img, 'mono16')
    observation["color"] = bridge.imgmsg_to_cv2(color_img, 'rgb8')
    observation["color_camera_info"] = color_camera_info
    observation["depth_camera_info"] = depth_camera_info
    
    return observation