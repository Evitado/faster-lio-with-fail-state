# Profiling faster-lio with Tracy

Every build of faster-lio (catkin and the nix package deployed to the vehicle) contains a dormant
[Tracy](https://github.com/wolfpld/tracy) client. With profiling off (the default) nothing of it runs: no thread, no
socket, each instrumented spot is a check of one bool. With profiling on, the node starts `tracy-capture` itself and
writes a `.tracy` file that you copy off the machine and open in the Tracy viewer.

## On the target

Switch it on with the launch args of `launch/evitado.launch`:

| arg | default | |
|---|---|---|
| `profiling_enable` | `$FASTER_LIO_PROFILING`, else `false` | record a trace |
| `profiling_save_path` | `$FASTER_LIO_PROFILING_PATH`, else `/home/evitado/ssd/odometry/profiling` | folder for the trace, created if missing |
| `profiling_port` | `8186` | localhost port between the node and its recorder |

The environment variables reach the node even when the stack launches `evitado.launch` from another launch file
without passing args (as `evitado_vdb_mapping/launch/mapping.launch` does):

```sh
FASTER_LIO_PROFILING=true roslaunch evitado_vdb_mapping mapping.launch
# or directly
roslaunch faster_lio evitado.launch profiling_enable:=true profiling_save_path:=/home/evitado/ssd/odometry/profiling
```

The node logs `profiling: recording to <file>` at startup. Stop the stack normally (Ctrl-C, roslaunch shutdown): the
node sends its last data, `tracy-capture` writes

```
<profiling_save_path>/faster_lio_<YYYY-mm-dd_HH-MM-SS>.tracy
<profiling_save_path>/faster_lio_<YYYY-mm-dd_HH-MM-SS>.capture.log
```

and exits. A killed node (SIGKILL) loses the trace. A trace takes roughly 15 MB per hour of driving.

Copy it off, e.g. `scp evitado@<vehicle>:/home/evitado/ssd/odometry/profiling/*.tracy .`

## Viewing

Open the file in a Tracy viewer built from the same commit as the client (protocol 72; the Tracy 0.11.1 in nixpkgs
cannot read it). From this package:

```sh
nix build -f profiling/tracy-tools.nix -o profiling/tracy-tools   # tracy (viewer), tracy-capture, tracy-csvexport
profiling/tracy-tools/bin/tracy faster_lio_<date>.tracy
python3 profiling/summarize.py faster_lio_<date>.tracy           # text summary: time per stage, cpu, memory
```

What the trace shows:

- **Frames:** one per processed scan, so frame times are scan latencies.
- **Zones:** the pipeline stages of each scan (preprocess, undistort, downsample, IEKF update, ObsModel, map update,
  publishers) and the ROS loop (`ros::spinOnce`, `idle (rate.sleep)`).
- **Plots:** `process cpu [cores]` (1.0 = one busy core), `process rss`, `process threads`, `system mem available`
  (what the rest of the stack has left), and per scan: points, effective features, IEKF iterations, nn searches, map
  voxels, lidar buffer length.

The recording is the faster-lio process; `system mem available` and the timing of `ros::spinOnce` / `idle` show how
it fits into the rest of the stack. Context switches and sampling are compiled out (they need root or kernel
tracing permissions), so the trace looks the same on every machine.

## On a desktop, from a bag

```sh
# roscore running; -t publishes an identity lidar -> base TF for bags without /tf_static
profiling/profile_bag.sh -b run.bag                                    # /main/ac_filtered_points, /main/imu
profiling/profile_bag.sh -b raw.bag -l /lidar1/points -i /lidar1/imu -t
```

It plays the bag in real time with profiling on, writes the trace to `Log/` (or `-d <dir>`) and prints the summary.
A catkin build finds the recorder at `profiling/tracy-tools/bin/tracy-capture` (build it as above), otherwise
`tracy-capture` from `PATH`; the nix package brings its own.
