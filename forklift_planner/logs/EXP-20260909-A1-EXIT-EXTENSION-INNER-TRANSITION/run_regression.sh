#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /mnt/d/desktop/叉车/devel/setup.bash
set -u

output_dir=${1:-/mnt/d/desktop/叉车/src/forklift_planner/logs/EXP-20260909-A1-EXIT-EXTENSION-INNER-TRANSITION}

roscore >/tmp/exp_a1_route_roscore.log 2>&1 &
ros_pid=$!
trap 'kill "$ros_pid" 2>/dev/null || true' EXIT

for _ in {1..50}; do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

rosparam load /mnt/d/desktop/叉车/src/forklift_planner/config/planner_param.yaml
rosrun forklift_planner path_curvature_audit \
  _output_dir:="$output_dir"
