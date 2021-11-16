#!/usr/bin/env bash
#
# Entry-point for app-scene-viewer
#
# Copyright 2021 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

source /opt/ros/*/setup.sh
source /opt/path.sh

# Start roscore for debugging
[[ $1 =~ ros* ]] || [[ $1 =~ run.sh ]] || [[ $1 =~ bash ]] || [[ $(rosnode list 2>/dev/null) ]] || roscore &

echo "Waiting for roscore to become ready..."
while true;
do
  sleep 1
  rosnode list > /dev/null 2>&1 && break
done

set -x

# Execute commands
exec "$@"
