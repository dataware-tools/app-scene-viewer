#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Utilities
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

from collections import OrderedDict
from copy import deepcopy
import math

from ai_instructor.msg import States, Checkpoint, Scenario, Attribute, Waypoint, Waypoints, Tag
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
import rospy
from std_msgs.msg import ColorRGBA
import tf
from visualization_msgs.msg import Marker, MarkerArray


class FailedToResolveTimestampsPassingCheckpointsError(Exception):
    pass


def add_offset_to_point(point, dx=0, dy=0, dz=0, only_if_zero=False, **kwargs):
    """Add offset to point.

    Args:
        point (geometry_msgs.msg.Point): pose
        dx (float): x offset
        dy (float): y offset
        dz (float): z offset
        only_if_zero (bool): add offset only if the value is zero

    Returns:
        (geometry_msgs.msg.Point): point

    """
    new_point = deepcopy(point)
    new_point.x += dx if (not only_if_zero) or point.x == 0.0 else 0
    new_point.y += dy if (not only_if_zero) or point.y == 0.0 else 0
    new_point.z += dz if (not only_if_zero) or point.z == 0.0 else 0
    return new_point


def add_offset_to_pose(pose, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0, only_if_zero=False, **kwargs):
    """Add offset to pose.

    Args:
        pose (geometry_msgs.msg.Pose): pose
        dx (float): x offset
        dy (float): y offset
        dz (float): z offset
        droll (float): roll offset
        dpitch (float): pitch offset
        dyaw (float): yaw offset
        only_if_zero (bool): add offset only if the value is zero

    Returns:
        (geometry_msgs.msg.Pose): pose

    """
    new_pose = deepcopy(pose)

    # Add offset to position
    new_pose.position.x += dx if (not only_if_zero) or pose.position.x == 0.0 else 0
    new_pose.position.y += dy if (not only_if_zero) or pose.position.y == 0.0 else 0
    new_pose.position.z += dz if (not only_if_zero) or pose.position.z == 0.0 else 0

    # Convert quaternion to euler angle
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([
        new_pose.orientation.x,
        new_pose.orientation.y,
        new_pose.orientation.z,
        new_pose.orientation.w,
    ])

    # Add offset to orientation
    roll += droll if (not only_if_zero) or roll == 0.0 else 0
    pitch += dpitch if (not only_if_zero) or pitch == 0.0 else 0
    yaw += dyaw if (not only_if_zero) or yaw == 0.0 else 0

    # Convert euler angle back to quaternion
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    new_pose.orientation.x = quaternion[0]
    new_pose.orientation.y = quaternion[1]
    new_pose.orientation.z = quaternion[2]
    new_pose.orientation.w = quaternion[3]

    # Return
    return new_pose


def state_dict_to_waypoints_msg(states, config):
    """Convert state dict to Waypoints and MarkerArray message containing waypoints.

    Args:
        states (dict): dict of states
        config (dict): dict of configs

    Returns:
        (Waypoints, MarkerArray): messages

    """
    def create_marker_msg(header, ns, id, pose, previous_pose=None, color='white'):

        marker = Marker()
        marker.header = header
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(secs=0, nsecs=0)

        # # For arrow
        # marker.type = marker.ARROW
        # pose_rotated = add_offset_to_pose(pose, dyaw=((-1.0) * math.pi / 2))     # Rotate for 90 degrees
        # marker.pose = add_offset_to_pose(pose_rotated, **config['visualization_offsets']['waypoints'])
        # marker.scale = Vector3(x=1.0, y=0.2, z=0.2)

        # For LINE_STRIP
        marker.type = marker.LINE_STRIP
        marker.pose = Pose(position=Point(), orientation=Quaternion(w=1.0))
        marker.scale = Vector3(x=0.50)
        if previous_pose is not None:
            marker.points.append(add_offset_to_point(previous_pose.position,
                                                     **config['visualization_offsets']['waypoints']))
            marker.points.append(add_offset_to_point(pose.position,
                                                     **config['visualization_offsets']['waypoints']))

        if color == 'white':
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.7)
        if color == 'red':
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
        if color == 'green':
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
        if color == 'blue':
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)
        if color == 'yellow':
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
        if color == 'cyan':
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.7)
        if color == 'purple':
            marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)

        return marker

    def create_waypoint_msg(id, pose, frame_id='map'):
        waypoint = Waypoint()
        waypoint.frame_id = frame_id
        waypoint.id = id
        waypoint.pose = pose
        return waypoint

    waypoints = Waypoints()
    markers = MarkerArray()
    initial_id = config['waypoint_initial_id'] if 'waypoint_initial_id' in config.keys() else 0
    if 'ego_poses' in states.keys():
        previous_pose = None
        poses = OrderedDict()
        if 'last_ego_pose' in states.keys():
            poses.update({rospy.Time(0): states['last_ego_pose']})
        poses.update(states['ego_poses'])

        for idx, (timestamp, ego_pose) in enumerate(poses.items(), initial_id):
            if previous_pose is None:
                previous_pose = ego_pose.pose
                continue
            else:
                distance_diff = math.sqrt((ego_pose.pose.position.x - previous_pose.position.x) ** 2
                                          + (ego_pose.pose.position.y - previous_pose.position.y) ** 2)
                if distance_diff < config['waypoint_distance_interval_to_visualize']:
                    continue

            marker = create_marker_msg(
                header=ego_pose.header,
                ns='waypoints',
                id=idx,
                pose=ego_pose.pose,
                previous_pose=previous_pose,
                color='blue'
            )
            waypoint = create_waypoint_msg(
                id=idx,
                pose=ego_pose.pose,
                frame_id='map'
            )
            markers.markers.append(marker)
            waypoints.points.append(waypoint)

            # visualize wheel waypoints
            if 'previous_wheel_tf' not in states.keys():
                states['previous_wheel_tf'] = {}
            if config['enable_wheel_waypoints']:
                base_frame_id = ego_pose.header.frame_id
                base_frame_id = base_frame_id[1:] if base_frame_id.startswith('/') else base_frame_id
                for wheel_idx, wheel_frame_id in enumerate(config['wheel_frame_ids']):
                    wheel_frame_id = wheel_frame_id[1:] if wheel_frame_id.startswith('/') else wheel_frame_id

                    try:
                        transform = states['tf_buffer'].lookup_transform(
                            base_frame_id, wheel_frame_id, rospy.Time(0),
                            rospy.Duration(0 if 'is_rosbag' in states.keys() and states['is_rosbag'] else 1)
                        )
                    except Exception as e:
                        print(e)
                        continue

                    if wheel_frame_id not in states['previous_wheel_tf'].keys():
                        states['previous_wheel_tf'][wheel_frame_id] = transform
                        continue

                    marker = create_marker_msg(
                        header=ego_pose.header,
                        ns='waypoints-{}'.format(wheel_frame_id),
                        id=idx,
                        pose=Pose(position=transform.transform.translation, orientation=transform.transform.rotation),
                        previous_pose=Pose(
                            position=states['previous_wheel_tf'][wheel_frame_id].transform.translation,
                            orientation=states['previous_wheel_tf'][wheel_frame_id].transform.rotation,
                        ),
                        color=['red', 'cyan', 'purple', 'yellow'][wheel_idx]
                    )
                    markers.markers.append(marker)
                    states['previous_wheel_tf'][wheel_frame_id] = transform

            previous_pose = ego_pose.pose

    return waypoints, markers


