#!/usr/bin/env bash
# Benchmark faster-lio on a bag in real time: launches evitado.launch, starts lidar odometry, plays the bag at its
# recorded rate and records a Tracy profile of the node, so cpu and memory usage match a live run.
#
#   profiling/profile_bag.sh -b <bag> [-o out.tracy] [-l lidar_topic] [-i imu_topic] [-t] [-r rate]
#     -l / -i  topics in the bag, remapped to the ones in config/ouster64.yaml
#              (default /main/ac_filtered_points and /main/imu, i.e. no remap)
#     -t       publish an identity base_footprint_tug -> main_sensor_lidar TF (bags without /tf_static)
#     -r       rosbag play rate (default 1.0, real time)
#
# Needs a running roscore and the node built with -DFASTER_LIO_TRACY=ON.
set -euo pipefail
# job control: background jobs keep a working SIGINT (non-interactive bash would start them with it ignored)
set -m

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bag=""
out="$here/../Log/faster_lio_$(date +%F_%H-%M-%S).tracy"
lidar_topic=/main/ac_filtered_points
imu_topic=/main/imu
fake_tf=0
rate=1.0
while getopts "b:o:l:i:tr:" opt; do
    case $opt in
        b) bag="$OPTARG" ;;
        o) out="$OPTARG" ;;
        l) lidar_topic="$OPTARG" ;;
        i) imu_topic="$OPTARG" ;;
        t) fake_tf=1 ;;
        r) rate="$OPTARG" ;;
        *) sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 1 ;;
    esac
done
if [[ -z "$bag" ]]; then
    sed -n '2,12p' "${BASH_SOURCE[0]}"
    exit 1
fi
if ! rosnode list > /dev/null 2>&1; then
    echo "no roscore reachable at $ROS_MASTER_URI" >&2
    exit 1
fi

pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill "$pid" 2> /dev/null || true; done
}
trap cleanup EXIT

if [[ $fake_tf == 1 ]]; then
    rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint_tug main_sensor_lidar > /dev/null 2>&1 &
    pids+=($!)
fi

capture="${TRACY_CAPTURE:-$here/tracy-tools/bin/tracy-capture}"
if [[ ! -x "$capture" ]]; then
    echo "tracy-capture not found, build it with: nix build -f $here/tracy-tools.nix -o $here/tracy-tools" >&2
    exit 1
fi
mkdir -p "$(dirname "$out")"

# recorder first: it waits for the node and records until the node disconnects
"$capture" -o "$out" -f > "${out%.tracy}.capture.log" 2>&1 &
capture_pid=$!

TRACY_NO_EXIT=1 roslaunch faster_lio evitado.launch > "${out%.tracy}.node.log" 2>&1 &
launch_pid=$!
pids+=($launch_pid)

echo "waiting for the node ..."
until rosservice list 2> /dev/null | grep -q "/lidar_odometry/start_lidar_odom"; do sleep 0.5; done
rosservice call /lidar_odometry/start_lidar_odom > /dev/null

echo "playing $bag (rate $rate)"
rosbag play -q -r "$rate" "$bag" "$lidar_topic:=/main/ac_filtered_points" "$imu_topic:=/main/imu"
sleep 2

kill -INT "$launch_pid"
wait "$launch_pid" || true
wait "$capture_pid" || true
echo "trace: $out"
python3 "$here/summarize.py" "$out"
