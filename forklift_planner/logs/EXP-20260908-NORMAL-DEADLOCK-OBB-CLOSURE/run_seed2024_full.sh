#!/usr/bin/env bash

port=11425
seed=2024
minutes=240
out_dir="/mnt/d/desktop/叉车/src/forklift_planner/logs/EXP-20260908-NORMAL-DEADLOCK-OBB-CLOSURE"

source /opt/ros/noetic/setup.bash
source "/mnt/d/desktop/叉车/devel/setup.bash"
set -u
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_LOG_DIR="/tmp/forklift_normal_deadlock_obb_seed2024_full_ros"
mkdir -p "${ROS_LOG_DIR}"

roscore -p "${port}" > "${out_dir}/seed2024_full_roscore.log" 2>&1 &
core_pid=$!
cleanup() { kill "${core_pid}" >/dev/null 2>&1 || true; }
trap cleanup EXIT

ready=0
for _ in {1..100}; do
  if rosparam list >/dev/null 2>&1; then ready=1; break; fi
  sleep 0.1
done
if [[ "${ready}" -ne 1 ]]; then exit 2; fi

rosparam load "/mnt/d/desktop/叉车/src/forklift_map/config/map_param.yaml"
rosparam load "/mnt/d/desktop/叉车/src/forklift_planner/config/planner_param.yaml"
rosparam set /use_sim_time false
rosparam set /forklift_planner/multi_vehicle/random_seed "${seed}"
rosparam set /forklift_planner/multi_vehicle/vehicle_count 2
rosparam set /forklift_planner/multi_vehicle/real_mode false
rosparam set /forklift_planner/multi_vehicle/a1_cycle_catalog_file "/mnt/d/desktop/叉车/src/forklift_planner/config/a1_cycle_path_catalog.yaml"

rosrun forklift_planner multi_vehicle_patrol_node \
  __name:=multi_vehicle_patrol_node \
  _target_only:=-1 _one_shot:=false _batch_minutes:="${minutes}" \
  _coord_log_file:="${out_dir}/seed2024_full_coord.log" \
  _stress_watchdog_enabled:=false _stress_quiet:=false \
  > "${out_dir}/seed2024_full_terminal.log" 2>&1
run_rc=$?
echo "SEED=${seed} MINUTES=${minutes} FULL_RUN_RC=${run_rc}"
exit "${run_rc}"
