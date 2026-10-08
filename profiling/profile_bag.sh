#!/usr/bin/env bash
# Profile faster-lio on a bag in real time: launches evitado.launch with profiling on, starts lidar odometry, plays
# the bag at its recorded rate, stops the node and prints a summary of the trace.
#
#   profiling/profile_bag.sh -b <bag> [-d save_dir] [-l lidar_topic] [-i imu_topic] [-t] [-r rate]
#     -d       folder for the .tracy file (default Log/ in this package)
#     -l / -i  topics in the bag, remapped to the ones in config/ouster128.yaml
#              (default /main/ac_filtered_points and /main/imu, i.e. no remap)
#     -t       publish an identity base_footprint_tug -> main_sensor_lidar TF (bags without /tf_static)
#     -r       rosbag play rate (default 1.0, real time)
#
# Needs a running roscore.
set -euo pipefail
set -m  # background jobs keep a working SIGINT

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bag=""
dir="$here/../Log"
lidar_topic=/main/ac_filtered_points
imu_topic=/main/imu
fake_tf=0
rate=1.0
while getopts "b:d:l:i:tr:" opt; do
    case $opt in
        b) bag="$OPTARG" ;;
        d) dir="$OPTARG" ;;
        l) lidar_topic="$OPTARG" ;;
        i) imu_topic="$OPTARG" ;;
        t) fake_tf=1 ;;
        r) rate="$OPTARG" ;;
        *) sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 1 ;;
    esac
done
[[ -n "$bag" ]] || { sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 1; }
rosnode list > /dev/null 2>&1 || { echo "no roscore reachable at $ROS_MASTER_URI" >&2; exit 1; }
dir="$(mkdir -p "$dir" && cd "$dir" && pwd)"

pids=()
trap 'for pid in "${pids[@]}"; do kill "$pid" 2> /dev/null || true; done' EXIT
if [[ $fake_tf == 1 ]]; then
    rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint_tug main_sensor_lidar > /dev/null 2>&1 &
    pids+=($!)
fi

before=$(date +%s)
roslaunch faster_lio evitado.launch profiling_enable:=true profiling_save_path:="$dir" > "$dir/profile_bag.node.log" 2>&1 &
launch_pid=$!
pids+=($launch_pid)
until rosservice list 2> /dev/null | grep -q "/lidar_odometry/start_lidar_odom"; do sleep 0.5; done
rosservice call /lidar_odometry/start_lidar_odom > /dev/null

echo "playing $bag (rate $rate)"
rosbag play -q -r "$rate" "$bag" "$lidar_topic:=/main/ac_filtered_points" "$imu_topic:=/main/imu"
sleep 2
kill -INT "$launch_pid"
wait "$launch_pid" || true

# the node's tracy-capture writes the file once the node has disconnected
for _ in $(seq 60); do
    pgrep -f "tracy-capture -o $dir/" > /dev/null || break
    sleep 1
done
trace=$(find "$dir" -maxdepth 1 -name 'faster_lio_*.tracy' -newermt "@$before" | sort | tail -1)
[[ -n "$trace" ]] || { echo "no trace written to $dir, see $dir/profile_bag.node.log" >&2; exit 1; }
echo "trace: $trace"
python3 "$here/summarize.py" "$trace"
