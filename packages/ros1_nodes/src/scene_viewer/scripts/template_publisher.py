#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Template publisher
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

import time

import roslib
import rospkg
roslib.load_manifest('diagnostic_updater')
import diagnostic_updater
import diagnostic_msgs
from std_msgs.msg import String

import rospy
import yaml

CONFIG_REQUIRED_KEYS = [
    'spin_rate',
]


class TemplatePublisher(object):
    """Template publisher."""

    def __init__(self):
        """Initialize handler."""
        super(TemplatePublisher, self).__init__()

        # initialize variables
        self.spin_rate = 2
        self.config = dict()
        self.diagnostics = {'is_error': False, 'reason': ''}

        # Initialize diagnostics updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("template_publisher")
        self.diagnostics_updater.add("status", self._diagnose)

        # initialize publishers
        self.publisher_topic = rospy.Publisher('~topic', String, queue_size=1, latch=True)

        try:
            # get ros-param
            rospy.logdebug("getting params")
            self._get_params()

            # load config
            rospy.logdebug("loading config")
            self._load_config()
            if 'spin_rate' in self.config.keys():
                self.spin_rate = self.config['spin_rate']

            # create a message
            rospy.logdebug('creating a message')
            msg = String()
            msg.data = 'template'

            # publish a message
            rospy.logdebug('publishing a message')
            self.publisher_topic.publish(msg)

        except Exception as e:
            rospy.logerr('error: ' + str(e))
            self.diagnostics['is_error'] = True
            self.diagnostics['reason'] = str(e)

    def _get_params(self):
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

    def _diagnose(self, stat):
        if self.diagnostics['is_error']:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, self.diagnostics['reason'])
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
        return stat

    def spin(self):
        """Continuously publish checkpoints."""
        while not rospy.is_shutdown():
            # update diagnostics
            self.diagnostics_updater.update()

            # sleep
            time.sleep(1.0 / self.spin_rate)


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('template_publisher', log_level=rospy.DEBUG)

    # for debugging
    import rosparam
    r = rospkg.RosPack()
    ns = 'template_publisher'
    if rospy.get_param('~path_to_config_file', None) is None:
        rosparam.set_param(ns + '/path_to_config_file',
                           r.get_path('scene_viewer') + '/conf/default.yaml')

    # create handler object
    handler = TemplatePublisher()

    # spin
    handler.spin()
