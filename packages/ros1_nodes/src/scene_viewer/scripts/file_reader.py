#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Template publisher
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

import json
import time
from typing import Optional

import roslib
roslib.load_manifest('diagnostic_updater')

import diagnostic_updater
import diagnostic_msgs
import pandas as pd
from pydtk.io import BaseFileReader
from pydtk.models import MetaDataModel
import rospkg
import rospy
from std_msgs.msg import String
import yaml


class FileReader(object):
    """Generic file reader."""

    def __init__(self, file_path: str, mapping: Optional[dict] = None, filtering: Optional[list] = None):
        """A generic file reader.
        Args:
            file_path (str): path to the file to load
            mapping (dict): a dictionary to map columns in the file
            filtering (list): a list to filter columns to publish

        """
        super(FileReader, self).__init__()
        self.data = ''
        self.file_path = file_path
        self.mapping = mapping
        self.filtering = filtering

        # load data
        df = self._read_file(file_path)
        df = self._map_columns(df, mapping)
        df = self._filter_columns(df, filtering)

        # convert to a list of dicts
        self.data = df.to_dict('records')

    @staticmethod
    def _read_file(path: str) -> pd.DataFrame:
        reader = BaseFileReader()
        _, _data, _columns = reader.read(MetaDataModel(data={"path": path}))
        data = _data[1:, :]
        columns = _data[0, :]

        # Convert to pandas dataframe
        df = pd.DataFrame(data, columns=columns)

        return df

    @staticmethod
    def _map_columns(df: pd.DataFrame, mapping: Optional[dict]) -> pd.DataFrame:
        mapping_dict = mapping or {}
        new_df = df.copy()
        for key, value in mapping_dict.items():
            new_df[value] = df[key]
            del new_df[key]
        return new_df

    @staticmethod
    def _filter_columns(df: pd.DataFrame, filtering: Optional[list]) -> pd.DataFrame:
        if filtering is None:
            return df
        new_df = df.copy()
        for column in new_df.columns:
            if column not in filtering:
                del new_df[column]
        return new_df

    def __str__(self):
        return json.dumps(self.data)


class FileReaderNode(object):
    """Generic file reader node."""

    def __init__(self):
        """A generic file reader node."""
        super(FileReaderNode, self).__init__()

        # initialize variables
        self.config = dict()
        self.diagnostics = {'is_error': False, 'reason': ''}

        # Initialize diagnostics updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("file_reader")
        self.diagnostics_updater.add("status", self._diagnose)

        # initialize publishers
        self.publisher_topic = rospy.Publisher('~content', String, queue_size=1, latch=True)

        try:
            # get ros-param
            rospy.logdebug("getting params")
            self._get_params()

            # load config
            rospy.logdebug("loading config")
            self._load_config()

            # initialize file reader
            rospy.logdebug('initializing file reader')
            self.reader = FileReader(
                self.path_to_file,
                mapping=self.config['file_reader']['mapping'],
                filtering=self.config['file_reader']['filtering'],
            )

            # create a message
            rospy.logdebug('creating a message')
            msg = String()
            msg.data = str(self.reader)

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
        self.path_to_file = rospy.get_param('~path_to_file', None)

    def _load_config(self):
        # load
        with open(self.path_to_config_file, 'r') as f:
            config = yaml.load(f, Loader=yaml.Loader)

        # check
        if 'file_reader' not in config.keys():
            raise KeyError('Key "file_reader" is missing in config')
        if 'mapping' not in config['file_reader'].keys():
            raise KeyError('Key "mapping" is missing in "file_reader" in config')
        if 'filtering' not in config['file_reader'].keys():
            raise KeyError('Key "filtering" is missing in "file_reader" in config')

        self.config = config

    def _diagnose(self, stat):
        if self.diagnostics['is_error']:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, self.diagnostics['reason'])
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
        return stat

    def spin(self):
        """Continuously update diagnostics."""
        while not rospy.is_shutdown():
            # update diagnostics
            self.diagnostics_updater.update()

            # sleep
            time.sleep(1)


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('file_reader', log_level=rospy.DEBUG)

    # for debugging
    import rosparam
    r = rospkg.RosPack()
    ns = 'file_reader'
    if rospy.get_param('~path_to_config_file', None) is None:
        rosparam.set_param(ns + '/path_to_config_file',
                           r.get_path('scene_viewer') + '/conf/default.yaml')
    if rospy.get_param('~path_to_file', None) is None:
        rosparam.set_param(ns + '/path_to_file',
                           r.get_path('scene_viewer') + '/sample/scene_caption.csv')

    # create handler object
    handler = FileReaderNode()

    # spin
    handler.spin()
