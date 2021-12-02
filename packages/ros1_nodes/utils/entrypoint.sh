#!/usr/bin/env bash
#
# Entry-point for app-scene-viewer
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

source /opt/ros1_nodes/devel/setup.bash

# Activate python3
[[ ! -d /tmp/python3 ]] && mkdir -p /tmp/python3
[[ ! -L /tmp/python3/python ]] && ln -s /usr/bin/python3 /tmp/python3/python
export PATH=/tmp/python3:${PATH}

exec "$@"
