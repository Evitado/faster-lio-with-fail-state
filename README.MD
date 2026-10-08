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

## Profiling

See [profiling/README.md](profiling/README.md) (Tracy, enabled with `-DFASTER_LIO_TRACY=ON`).

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
