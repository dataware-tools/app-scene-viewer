#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Topic aggregator
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

import json
import time

import roslib
roslib.load_manifest('diagnostic_updater')

import diagnostic_updater
import diagnostic_msgs
import pandas as pd
import rospkg
import rospy
from std_msgs.msg import String
import yaml


class ConfigError(Exception):
    """Error when config file is wrong."""


class TopicAggregator(object):
    """Topic aggregator."""

    def __init__(self, config: dict, messages: dict):
        """A topic aggregator."""
        super(TopicAggregator, self).__init__()
        self.dfs = {}
        self.data = ''
        self.config = config
        self.messages = messages

        # check
        assert set(config['input']['topics'].keys()) == set(messages.keys())

        # parse
        self._parse_messages()

        # aggregate
        self._aggregate_messages()

    def _parse_messages(self):
        topic_dfs = {}
        for topic, topic_config in self.config['input']['topics'].items():
            msg = json.loads(self.messages[topic].data)
            if topic_config['dtype'] == 'dict':
                topic_dfs[topic] = pd.DataFrame.from_records(msg, index='timestamp')
            elif topic_config['dtype'] == 'list':
                topic_dfs[topic] = pd.DataFrame.from_records(msg, columns=topic_config['columns'], index='timestamp')
            else:
                raise ValueError('Unsupported dtype: {}'.format(topic_config['dtype']))
            topic_dfs[topic].index = topic_dfs[topic].index.map(float)
        self.dfs = topic_dfs

    def _aggregate_messages(self):
        # TODO: find the neighboring timestamp and aggregate them (use DataFrame.interpolate)
        df = pd.concat(self.dfs.values(), axis=1, join='outer')
        df_interpolated = df.interpolate(method='nearest')
        indices = [idx in self.dfs[self.config['input']['base_topic']].index for idx in df_interpolated.index]
        self.data = df_interpolated[indices].reset_index().to_dict('record')

    def __str__(self):
        return json.dumps(self.data)


class TopicAggregatorNode(object):
    """Topic aggregator node."""

    def __init__(self):
        """A topic aggregator node."""
        super(TopicAggregatorNode, self).__init__()

        # initialize variables
        self.config = dict()
        self.diagnostics = {'is_error': False, 'reason': ''}

        # Initialize diagnostics updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("topic_aggregator")
        self.diagnostics_updater.add("status", self._diagnose)

        # initialize publishers
        self.publisher_topic = rospy.Publisher('~content', String, queue_size=1, latch=True)

        # initialize subscribers
        self.subscribers = {}
        self.messages = {}

        try:
            # get ros-param
            rospy.logdebug("getting params")
            self._get_params()

            # load config
            rospy.logdebug("loading config")
            self._load_config()

            # initialize subscribers
            self._prepare_subscribers()

            # wait until all the topics are subscribed
            self._wait_for_subscription()

            # initialize topic aggregator
            rospy.logdebug('initializing topic aggregator')
            self.aggregator = TopicAggregator(
                config=self.config['topic_aggregator'],
                messages=self.messages
            )

            # create a message
            rospy.logdebug('creating a message')
            msg = String()
            msg.data = str(self.aggregator)

            # publish a message
            rospy.logdebug('publishing a message')
            self.publisher_topic.publish(msg)

        except Exception as e:
            reason = 'error (' + type(e).__name__ + '): ' + str(e)
            rospy.logerr(reason)
            self.diagnostics['is_error'] = True
            self.diagnostics['reason'] = reason

    def _get_params(self):
        self.path_to_config_file = rospy.get_param('~path_to_config_file', None)

    def _load_config(self):
        # load
        with open(self.path_to_config_file, 'r') as f:
            config = yaml.load(f, Loader=yaml.Loader)

        # check
        if 'topic_aggregator' not in config.keys():
            raise KeyError('Key "topic_aggregator" is missing in config')
        if 'input' not in config['topic_aggregator'].keys():
            raise KeyError('Key "input" is missing in "topic_aggregator" in config')
        if 'base_topic' not in config['topic_aggregator']['input'].keys():
            raise KeyError('Key "base_topic" is missing in "topic_aggregator.input" in config')
        if 'topics' not in config['topic_aggregator']['input'].keys():
            raise KeyError('Key "topics" is missing in "topic_aggregator.input" in config')
        for topic, topic_config in config['topic_aggregator']['input']['topics'].items():
            if 'dtype' not in topic_config.keys():
                raise KeyError('Key "dtype" is missing in "topic_aggregator.input.topics.{}"'.format(topic))
            assert topic_config['dtype'] in ['dict', 'list'], 'dtype must be either "dict" or "list"'
            if topic_config['dtype'] == 'list':
                if 'columns' not in topic_config.keys():
                    raise KeyError('Key "columns" is missing in "topic_aggregator.input.topics.{}"'.format(topic))
        if 'output' not in config['topic_aggregator'].keys():
            raise KeyError('Key "output" is missing in "topic_aggregator" in config')
        if 'dtype' not in config['topic_aggregator']['output'].keys():
            raise KeyError('Key "dtype" is missing in "topic_aggregator.output" in config')
        if 'columns' not in config['topic_aggregator']['output'].keys():
            raise KeyError('Key "columns" is missing in "topic_aggregator.output" in config')

        self.config = config

    def _diagnose(self, stat):
        if self.diagnostics['is_error']:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, self.diagnostics['reason'])
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
        return stat

    def _prepare_subscribers(self):
        for topic in self.config['topic_aggregator']['input']['topics'].keys():
            self.messages[topic] = None
            self.subscribers[topic] = rospy.Subscriber(topic, String, self._callback, topic)
        if len(self.messages.keys()) == 0:
            raise ConfigError(
                'Topics to input is missing in config. '
                'Please make sure more than one topics exist in "topic_aggregator.input.topics"'
            )

    def _callback(self, msg, topic):
        rospy.logdebug(topic)
        self.messages[topic] = msg

    def _wait_for_subscription(self):
        while not rospy.is_shutdown():
            if all([msg is not None for msg in self.messages.values()]):
                break
            time.sleep(1)

    def spin(self):
        """Continuously update diagnostics."""
        while not rospy.is_shutdown():
            # update diagnostics
            self.diagnostics_updater.update()

            # sleep
            time.sleep(1)


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('topic_aggregator', log_level=rospy.DEBUG)

    # for debugging
    import rosparam
    r = rospkg.RosPack()
    ns = 'topic_aggregator'
    if rospy.get_param('~path_to_config_file', None) is None:
        rosparam.set_param(ns + '/path_to_config_file',
                           r.get_path('scene_viewer') + '/conf/default.yaml')

    # create handler object
    handler = TopicAggregatorNode()

    # spin
    handler.spin()
