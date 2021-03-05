from pybullet_planning import  multiply, get_link_pose, joint_from_name, set_joint_position, joints_from_names, \
    set_joint_positions, get_joint_positions, get_min_limit, get_max_limit, quat_from_euler, read_pickle, set_pose, \
    get_pose, euler_from_quat, link_from_name, has_link, point_from_pose, invert, Pose, \
    unit_pose, joints_from_names, PoseSaver, get_aabb, get_joint_limits, get_joints, \
    ConfSaver, get_bodies, create_mesh, remove_body, single_collision, unit_from_theta, angle_between, violates_limit, \
    violates_limits, add_line, get_body_name, get_num_joints, approximate_as_cylinder, \
    approximate_as_prism, unit_quat, unit_point, clip, get_joint_info, tform_point, get_yaw, \
    get_pitch, wait_for_user, quat_angle_between, angle_between, quat_from_pose, compute_jacobian, \
    movable_from_joints, quat_from_axis_angle, LockRenderer, Euler, get_links, get_link_name,\
    draw_point, draw_pose, get_extend_fn, get_moving_links, link_pairs_collision, draw_point, get_link_subtree, \
    clone_body, get_all_links, set_color, pairwise_collision, tform_point,  wrap_angle

 
import math
import os
import random
import re
from collections import namedtuple
from itertools import combinations

import numpy as np


GRASP_LENGTH = 0.
TOOL_POSE = Pose(euler=Euler(pitch=np.pi/2)) 
MAX_GRASP_WIDTH = np.inf

SIDE_HEIGHT_OFFSET = 0.03 # z distance from top of object

def get_top_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(), max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    # TODO: rename the box grasps
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    # h-=0.1
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, h / 2 - grasp_length])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, math.pi / 2 + i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    return grasps

def get_side_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(), max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH, top_offset=SIDE_HEIGHT_OFFSET):
    # TODO: compute bounding box width wrt tool frame
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    # h-=0.1
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    #x_offset = 0
    x_offset = h/2 - top_offset
    for j in range(1 + under):
        swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
        if w <= max_width:
            translate_z = Pose(point=[x_offset, 0, l / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[math.pi / 2 + i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([w])
        if l <= max_width:
            translate_z = Pose(point=[x_offset, 0, w / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([l])
    return grasps


# Cylinder grasps

def get_top_cylinder_grasps(body, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                            max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    # Apply transformations right to left on object pose
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    height-=0.1
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, height / 2 - grasp_length])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    if max_width < diameter:
        return
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(euler=[0, 0, theta])
        yield multiply(tool_pose, translate_z, rotate_z,
                       reflect_z, translate_center, body_pose)

def get_side_cylinder_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                             max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH,
                             top_offset=SIDE_HEIGHT_OFFSET):
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    # height-=0.1
    translate_center = Pose(point_from_pose(body_pose)-center)
    #x_offset = 0
    x_offset = height/2 - top_offset
    if max_width < diameter:
        return
    while True:
        theta = random.uniform(0, 2*np.pi)
        translate_rotate = ([x_offset, 0, diameter / 2 - grasp_length], quat_from_euler([theta, 0, 0]))
        for j in range(1 + under):
            swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
            yield multiply(tool_pose, translate_rotate, swap_xz, translate_center, body_pose)

def get_edge_cylinder_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                             grasp_length=GRASP_LENGTH):
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    # height-=0.1
    translate_yz = Pose(point=[0, diameter/2, height/2 - grasp_length])
    reflect_y = Pose(euler=[0, math.pi, 0])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(euler=[0, 0, theta])
        for i in range(1 + under):
            rotate_under = Pose(euler=[0, 0, i * math.pi])
            yield multiply(tool_pose, rotate_under, translate_yz, rotate_z,
                           reflect_y, translate_center, body_pose)