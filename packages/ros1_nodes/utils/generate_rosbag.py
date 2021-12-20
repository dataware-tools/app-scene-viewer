#!/usr/bin/env python3

"""ros1bag generator."""

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg._NavSatFix import NavSatFix

import os
import pathlib


def generate_ros1bag(ros1bag: str):
    """Generate a dummy ros1bag including location messages."""

    topic = "/sensing/gnss/ublox/nav_sat_fix"
    locs = [
        [35.144697, 136.964871],
        [35.146017695754104, 136.9644168258311],
        [35.15154533435691, 136.96603391120726],
        [35.15319804374357, 136.96580931601613],
        [35.15447, 136.966588],
    ]
    start_time = 1638360000.0  # 2021.12.01-12:00:00
    end_time = start_time + float(len(locs)) - 1

    time = start_time
    with rosbag.Bag(ros1bag, "w") as out_bag:
        while time < end_time:
            latitude = np.interp(time, [idx + start_time for idx in range(len(locs))], [loc[0] for loc in locs])
            longitude = np.interp(time, [idx + start_time for idx in range(len(locs))], [loc[1] for loc in locs])
            msg = NavSatFix(
                latitude=latitude,
                longitude=longitude,
                altitude=0.
            )
            ros_time = rospy.Time.from_sec(time)
            out_bag.write(topic, msg, ros_time)
            time += 0.1


if __name__ == "__main__":
    rosbag_path = "/opt/ros1_nodes/sample_data/sample.bag"
    done_file = rosbag_path + ".done"
    if os.path.isfile(done_file):
        print("Already generated!")
    else:
        print("Generating rosbag...")
        generate_ros1bag(rosbag_path)
        done_path = pathlib.Path(done_file)
        done_path.touch()
        print("Done generating!")
