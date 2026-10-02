# Profiling faster-lio with Tracy

faster-lio can be instrumented with [Tracy](https://github.com/wolfpld/tracy), the profiler evitado_common already
vendors (`evitado_common/foreign/tracy`). A run is recorded into a `.tracy` file that you open afterwards in the Tracy
viewer. Nothing has to be watched live.

The recording shows:

- **Zones:** per-scan timeline of every pipeline stage (preprocess, undistort, downsample, IEKF, ObsModel, map update,
  publishers), including the TBB worker threads.
- **Frames:** one frame per processed scan, so frame times are scan latencies.
- **Plots:**
  - `process cpu [cores]`: CPU used by the node, where 1.0 means one fully busy core.
  - `CPU usage`: Tracy's whole-machine CPU, all processes.
  - `process rss`, `process threads`, and `system mem available` (what is left for everything else).
  - Per-scan point counts, effective features, IEKF iterations and map voxels.
- **Memory:** every `operator new`/`delete` of the process, with live memory over time. Eigen-aligned buffers
  (pcl point storage) use `malloc` and are not tracked individually; `process rss` covers them.

## 1. Build the capture tools (once)

```sh
nix build -f profiling/tracy-tools.nix -o profiling/tracy-tools
```

This builds `tracy-capture`, `tracy-csvexport` (and a viewer, `tracy`) from the same Tracy commit as the client.
The Tracy 0.11.1 packaged in nixpkgs uses an older protocol and cannot read these traces.

## 2. Build faster-lio with profiling

```sh
catkin build faster_lio --cmake-args -DFASTER_LIO_TRACY=ON    # profiling build
catkin build faster_lio --cmake-args -DFASTER_LIO_TRACY=OFF   # back to the normal build (the option is cached)
```

`-DFASTER_LIO_TRACY_MEMORY=OFF` keeps the zones but drops the per-allocation memory tracking.

In a profiling build, Tracy buffers data in memory until a recorder connects, so do not deploy it. The node logs
`PROFILING BUILD` at startup.

## 3. Record

Benchmark on a bag in real time. This launches `evitado.launch`, starts LIO, plays the bag and records:

```sh
# roscore must be running
profiling/profile_bag.sh -b run.bag                                   # bag with /main/ac_filtered_points, /main/imu
profiling/profile_bag.sh -b raw.bag -l /lidar1/points -i /lidar1/imu -t   # other topics; -t: identity lidar TF
```

Or record any way of starting the node, for example the full stack on the vehicle:

```sh
profiling/record.sh -o lio.tracy -- roslaunch evitado_vdb_mapping mapping.launch
```

Stop with Ctrl-C. Either way, the trace lands in `Log/faster_lio_<date>.tracy` (or `-o`) and a text summary is
printed. `python3 profiling/summarize.py <file>.tracy` reprints it.

## 4. View

Open the `.tracy` file in a Tracy viewer **0.12 or newer** (File → Open), for example a release build from
<https://github.com/wolfpld/tracy/releases> on any machine. Useful views:

- **Timeline:** scans as frames, with stages nested under `LaserMapping::Run` and TBB workers on their own rows.
- **Find zone / Statistics:** time distribution per stage.
- **Plots** under the timeline: cpu, rss, available memory, point counts.
- **Memory:** allocations, live memory and leaks over time.

### Seeing other processes

Tracy can also record context switches and which process ran on each core, to show who faster-lio competes with.
That needs kernel tracing permissions for the profiled process: run the node as root, or relax them for the session
with `sudo sysctl kernel.perf_event_paranoid=-1` and read access to `/sys/kernel/tracing`. Without them, the CPU
usage plots above still work.
