#!/usr/bin/env bash
#
# Get sample rosbag
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi
#

set -x

download_url="https://autoware-ai.s3.us-east-2.amazonaws.com/sample_moriyama_150324.tar.gz"
rosbag_path=/opt/ros_nodes/sample_data/sample.bag
done_file=${rosbag_path}.done

[[ -f ${done_file} ]] && echo "Already done" && exit 0

cd $(dirname ${rosbag_path})
[[ ! -f sample.tar.gz ]] && wget -O sample.tar.gz ${download_url}
[[ ! -f sample_moriyama_150324.bag ]] && tar zxf sample.tar.gz
[[ ! -L $(basename ${rosbag_path}) ]] && ln -s sample_moriyama_150324.bag $(basename ${rosbag_path})
touch ${done_file}