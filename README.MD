## FasterLIO (Evitado fork)

Lidar-inertial odometry for the tug's main ouster lidar, based on
[Faster-LIO](https://github.com/gaoxiang12/faster-lio) (itself built on [FastLIO2](https://github.com/hku-mars/FAST_LIO)).
This fork keeps only what the Evitado stack runs: online ROS mode with an ouster-layout point cloud.

## Build

Built as part of the evitado nix workspace (`src/faster_lio/package.nix`), e.g. `nix build --impure .#faster-lio`
from the workspace root. Dependencies: ROS noetic (roscpp, tf, nav_msgs, sensor_msgs, std_srvs), Eigen and TBB.

## Run

`evitado_vdb_mapping/launch/mapping.launch` includes `launch/evitado.launch`, which loads `config/ouster128.yaml` and
starts `run_mapping_online` as `lidar_odometry`.

- Subscribes: `common/lid_topic` (PointCloud2, ouster fields; without a `t` field all points are treated as
  simultaneous) and `common/imu_topic`.
- Publishes (`~`): `odometry`, `trajectory`, `keypoints`, `condition_number`, and the TF `global_frame` -> `base_link_frame`.
- Services (`~`): `start_lidar_odom` / `stop_lidar_odom`. While stopped, odometry stays at the origin.
- `~save_trajectory` (`faster_lio/SaveTrajectory`): writes every scan's pose since the last `start_lidar_odom` to a
  TUM file (`timestamp tx ty tz qx qy qz qw`, readable by `evo_traj tum`). The trajectory is kept after
  `stop_lidar_odom`, so it can be saved afterwards. An empty `file_path` uses the `~trajectory_file` parameter:

  ```bash
  rosservice call /lidar_odometry/save_trajectory "file_path: '/tmp/run.tum'"
  ```

### Self-stop and restart

The odometry stops itself, and then publishes as stopped until `start_lidar_odom`, in two cases:

- `max_skipped_scan_time` (0.5 s) of scans in a row without a filter update: too few points, or no point-to-plane
  correspondence.
- The filter diverged (non-finite state or covariance). It is caught before anything is published, and the filter is
  reset to the origin with the startup IMU biases, so the stopped keypoints stay finite.

**Known gap:** faster_lio never restarts itself; `odometry_fail_state` (`evitado_vdb_mapping`) has to call
`start_lidar_odom`. A self-restart would skip what fail_state does around it: resetting aircraft_state_estimation and
restarting ohm_mapping for the new odom origin. But fail_state only restarts LIO after it saw a failure itself: 10
failing frames in a row (`sensor_frequency`), checked in fixed blocks of 10, so ~1-2 s. While LIO is stopped its
odometry is identity, which fail_state takes as a standing tug, and it clears its buffer. So LIO stays stopped,
until something else calls `start_lidar_odom`, after:

- a blinding that lasts longer than `max_skipped_scan_time` but too short for fail_state to see (up to ~2 s);
- a divergence while the lidar is healthy: fail_state never sees a failure.

Both are rare. The fix belongs in fail_state: when the odometry reports stopped (`child_frame_id` is
`base_link_frame`) and the lidar is healthy, run its normal stop/start sequence. Raising `max_skipped_scan_time` above
~2 s also closes the first case, at the cost of publishing up to that long of IMU-only drift as live odometry.

## Profiling

Set `profiling_enable:=true` (or `FASTER_LIO_PROFILING=true`) to record a Tracy trace into `profiling_save_path`
(default `/home/evitado/ssd/odometry/profiling`); see [profiling/README.md](profiling/README.md).

## Citation

```
@ARTICLE{9718203,
author={Bai, Chunge and Xiao, Tao and Chen, Yajie and Wang, Haoqian and Zhang, Fang and Gao, Xiang},
journal={IEEE Robotics and Automation Letters},
title={Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels},
year={2022},
volume={7},
number={2},
pages={4861-4868},
doi={10.1109/LRA.2022.3152830}}
```
