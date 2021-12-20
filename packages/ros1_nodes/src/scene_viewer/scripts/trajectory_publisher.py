#!/usr/bin/env python3
#
# Trajectory publisher
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
#
import json

import rosbag
import roslib
import rospkg
roslib.load_manifest('diagnostic_updater')
import diagnostic_updater
from std_msgs.msg import String

import rospy

from template_publisher import TemplatePublisher


class TrajectoryPublisher(TemplatePublisher):
    """Trajectory publisher."""

    def __init__(self):
        """Initialize handler."""

        # initialize variables
        self.spin_rate = 2
        self.config = dict()
        self.diagnostics = {'is_error': False, 'reason': ''}

        # Initialize diagnostics updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("trajectory_publisher")
        self.diagnostics_updater.add("status", self._diagnose)

        # initialize publishers
        self.publisher_topic = rospy.Publisher('~info', String, queue_size=1, latch=True)

        try:
            # get ros-param
            rospy.logdebug("getting params")
            self._get_params()

            # load config
            rospy.logdebug("loading config")
            self._load_config()
            if 'spin_rate' in self.config.keys():
                self.spin_rate = self.config['spin_rate']

            # load rosbag
            self.traj_info = get_trajectory_info(self.path_to_rosbag, self.gnss_topic)

            # create a message
            rospy.logdebug('creating a message')
            msg = String()
            msg.data = self.traj_info

            # publish a message
            rospy.logdebug('publishing a message')
            self.publisher_topic.publish(msg)

        except Exception as e:
            rospy.logerr('error: ' + str(e))
            self.diagnostics['is_error'] = True
            self.diagnostics['reason'] = str(e)

    def _get_params(self):
        super()._get_params()
        self.path_to_rosbag = rospy.get_param('~path_to_rosbag', None)
        self.gnss_topic = rospy.get_param('~gnss_topic', '/sensing/gnss/ublox/nav_sat_fix')


support_types = [
    "sensor_msgs/NavSatFix",
]


def get_trajectory_info(bag: str, topic: str):
    """Get trajectory information from rosbag."""
    posts = []
    with rosbag.Bag(bag, "r") as b:
        types, topics = b.get_type_and_topic_info()
        if topic not in topics.keys():
            rospy.logerr(f"error: topic '{topic}' is not in the {bag} .")
        if topics[topic][0] not in support_types:
            rospy.logerr(f"error: tpic_type '{topics[topic][0]}' is not supported.")
        for _, msg, t in b.read_messages(topics=topic):
            posts += [[t.to_sec(), msg.latitude, msg.longitude, msg.altitude]]
        # posts = resample_trajectory(trjs)
    traj_str = json.dumps(posts)
    return traj_str


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('trajectory_publisher', log_level=rospy.DEBUG)

    # for debugging
    import rosparam
    r = rospkg.RosPack()
    ns = 'trajectory_publisher'
    if rospy.get_param('~path_to_config_file', None) is None:
        rosparam.set_param(ns + '/path_to_config_file',
                           r.get_path('scene_viewer') + '/conf/default.yaml')
    if rospy.get_param('~path_to_rosbag', None) is None:
        rosparam.set_param(ns + '/path_to_rosbag',
                           r.get_path('scene_viewer') + '/sample/sample.bag')
    if rospy.get_param('~gnss_topic', None) is None:
        rosparam.set_param(ns + '/gnss_topic', '/sensing/gnss/ublox/nav_sat_fix')

    # create handler object
    handler = TrajectoryPublisher()

    # spin
    handler.spin()
