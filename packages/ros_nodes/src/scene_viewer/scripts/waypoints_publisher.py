#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Waypoints publisher
#
# Copyright 2020 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

from collections import OrderedDict
import math
import time

import roslib
roslib.load_manifest('diagnostic_updater')
import diagnostic_updater
import diagnostic_msgs
from geometry_msgs.msg import PoseStamped,Pose, Point, Vector3, Quaternion
from visualization_msgs.msg import MarkerArray

import rosbag
import rospy
import tf2_ros
import yaml

from utils import state_dict_to_waypoints_marker_aray_msg

CONFIG_REQUIRED_KEYS = [
    'spin_rate',
    'topic_ego_pose',
    'waypoint_initial_id',
    'waypoint_distance_interval_to_visualize',
    'enable_wheel_waypoints',
    'visualization_offsets'
]


def load_rosbag_file(path, config):
    """Load a rosbag file.

    Args:
        path (str): path to rosbag
        config (dict): config

    Returns:
        (dict): rosbag content

    """
    ego_poses = OrderedDict()
    tf_buffer = tf2_ros.Buffer()
    with rosbag.Bag(path, 'r') as bag:
        start_timestamp = bag.get_start_time()
        end_timestamp = bag.get_end_time()
        for topic, msg, t in bag.read_messages(topics=[config['topic_ego_pose'], '/tf']):
            if topic == '/tf':
                for transform in msg.transforms:
                    transform.transform.translation.__class__ = Vector3
                    transform.transform.rotation.__class__ = Quaternion
                    tf_buffer.set_transform(transform, '')

            if topic == config['topic_ego_pose']:
                ego_poses.update({t: msg})

    # check
    if len(ego_poses.values()) == 0:
        raise ValueError('No topic named "{}" found in rosbag'.format(config['topic_ego_pose']))

    info = {
        'is_rosbag': True,
        'ego_poses': ego_poses,
        'rosbag_start_timestamp': start_timestamp,
        'rosbag_end_timestamp': end_timestamp,
        'tf_buffer': tf_buffer
    }
    return info


class WaypointsPublisher(object):
    """Handler for waypoints of the ego-vehicle."""

    def __init__(self):
        """Initialize handler."""
        super(WaypointsPublisher, self).__init__()

        # initialize variables
        self.spin_rate = 2
        self.config = dict()
        self.diagnostics = {'is_error': False, 'reason': ''}
        self._init_states()

        # Initialize diagnostics updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("waypoints_publisher")
        self.diagnostics_updater.add("status", self._diagnose)

        # initialize publishers
        self.publisher_markers = rospy.Publisher('~markers', MarkerArray, queue_size=1, latch=True)

        try:
            # get ros-param
            self._get_params()

            # load config
            self._load_config()
            if 'spin_rate' in self.config.keys():
                self.spin_rate = self.config['spin_rate']

            # load rosbag
            if self.path_to_rosbag is not None:
                self._load_rosbag()

            # publish waypoints
            markers_msg = self._get_msg_to_publish()
            if markers_msg is not None:
                self.publisher_markers.publish(markers_msg)

            # clean states
            self._init_states()

            # initialize subscriber
            self.subscriber = rospy.Subscriber(self.config['topic_ego_pose'], PoseStamped, self._callback)

        except Exception as e:
            self.diagnostics['is_error'] = True
            self.diagnostics['reason'] = str(e)

    def _get_params(self):
        self.path_to_rosbag = rospy.get_param('~path_to_rosbag', None)
        self.path_to_config_file = rospy.get_param('~path_to_config_file', None)

    def _load_config(self):
        # load
        with open(self.path_to_config_file, 'r') as f:
            config = yaml.load(f, Loader=yaml.Loader)
        # check
        for key in CONFIG_REQUIRED_KEYS:
            if key not in config.keys():
                raise KeyError('Missing the following key in config file: {}'.format(key))
        # substitute
        self.config = config

    def _load_rosbag(self):
        info = load_rosbag_file(self.path_to_rosbag, self.config)
        self.states.update(info)

    def _init_states(self):
        self.states = dict()
        tf_buffer = tf2_ros.Buffer()
        self.states['tf_buffer'] = tf_buffer
        self.states['tf_listener'] = tf2_ros.TransformListener(tf_buffer)

    def _diagnose(self, stat):
        if self.diagnostics['is_error']:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, self.diagnostics['reason'])
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
        return stat

    def _get_msg_to_publish(self):
        if len(self.ego_poses) != 0:
            markers = state_dict_to_waypoints_marker_aray_msg(self.states, self.config)
            return markers
        return None, None

    def _callback(self, msg):
        is_ignore = False

        # check if current pose is far from the previously published pose enough to publish it
        if 'last_ego_pose' in self.states.keys():
            last_msg = self.states['last_ego_pose']
            distance_diff = math.sqrt((msg.pose.position.x - last_msg.pose.position.x) ** 2
                                      + (msg.pose.position.y - last_msg.pose.position.y) ** 2)
            if distance_diff < self.config['waypoint_distance_interval_to_visualize']:
                is_ignore = True

        # substitute current pose
        if not is_ignore:
            self.states['ego_poses'] = {rospy.Time.now(): msg}

    def spin(self):
        """Continuously publish checkpoints."""
        while not rospy.is_shutdown():
            # update diagnostics
            self.diagnostics_updater.update()

            if not self.diagnostics['is_error']:
                if self.path_to_rosbag is None:
                    # online mode (publish waypoints)
                    markers = self._get_msg_to_publish()
                    if markers is not None:
                        self.publisher_markers.publish(markers)
                        self.states['last_ego_pose'] = list(self.states['ego_poses'].values())[0]
                        del self.states['ego_poses']
                        self.config['waypoint_initial_id'] += 1

            # sleep
            time.sleep(1.0 / self.spin_rate)

    @property
    def ego_poses(self):
        """Return ego_poses."""
        if 'ego_poses' not in self.states.keys():
            self.states['ego_poses'] = OrderedDict()
        return self.states['ego_poses']


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('waypoints_publisher', log_level=rospy.ERROR)

    # for debugging
    import rosparam
    ns = 'waypoints_publisher'
    if rospy.get_param('~path_to_rosbag', None) is None:
        rosparam.set_param(ns + '/path_to_rosbag',
                           '/opt/samples/sample.bag')
    if rospy.get_param('~path_to_config_file', None) is None:
        rosparam.set_param(ns + '/path_to_config_file',
                           '/opt/ros_nodes/src/scene_viewer/conf/default.yaml')

    # create handler object
    handler = WaypointsPublisher()

    # spin
    handler.spin()
