import pyglet
pyglet.options['shadow_window'] = False
import os
import numpy as np
import trimesh
from tf import transformations

from pyrender import IntrinsicsCamera,\
                     DirectionalLight, SpotLight, PointLight,\
                     MetallicRoughnessMaterial,\
                     Primitive, Mesh, Node, Scene,\
                     Viewer, OffscreenRenderer

from grasp_proposal_from_mesh import *

def pose_matrix(euler, translation):
    pose_matrix = transformations.euler_matrix(euler[0], euler[1], euler[2])
    pose_matrix[:3,3] = translation
    return np.asarray(pose_matrix)

def rotate_vector(euler, vector):
    return np.matmul(transformations.euler_matrix(euler[0], euler[1], euler[2])[:3,:3],vector)

def generate_image(object_trimesh):
    object_face_colors = np.asarray([[1.,0.,0.]])*np.ones(object_trimesh.faces.shape)
    object_trimesh.visual.face_colors = object_face_colors
    object_mesh = Mesh.from_trimesh(object_trimesh, smooth=False)
    object_pose = pose_matrix(np.pi*(2.*np.random.uniform(size=[3,])-1.),[0.,0.,0.1])
    
    point_l = PointLight(color=np.ones(3), intensity=10.0)

    cam_width = 640
    cam_height = 480
    cam_fx = 521.179233
    cam_fy = 493.033034
    cam_cx = cam_width/2.
    cam_cy = cam_height/2.
    cam = IntrinsicsCamera(cam_fx,cam_fy,cam_cx,cam_cy)

    cam_euler = np.pi/12.*(2.*np.random.uniform(size=[3,])-1.)
    cam_euler[2] = 0.
    cam_pose = pose_matrix(cam_euler,rotate_vector(cam_euler,[0.,0.,1.0]))

    wood_trimesh = trimesh.load('./background_models/wood.obj')
    wood_trimesh.vertices = wood_trimesh.vertices*10.
    wood_mesh = Mesh.from_trimesh(wood_trimesh)

    scene = Scene(ambient_light=np.array([0.02, 0.02, 0.02, 10.0]))

    object_node = scene.add(object_mesh, pose=object_pose)
    point_l_node = scene.add(point_l, pose=cam_pose)
    wood_node = scene.add(wood_mesh)
    cam_node = scene.add(cam, pose=cam_pose)

    r = OffscreenRenderer(viewport_width=cam_width, viewport_height=cam_height,)
    color, depth = r.render(scene)
    r.delete()
    return color, depth, cam_pose, cam, object_pose