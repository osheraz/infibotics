# Infi navigation package

implementation of various slam & navigation packages

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

``` bash
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```
``` bash
$ https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation
```



### Carto parameters:

Lua configuration reference documentation
```
$ https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html```
```

####Inputs:

```num_laser_scans``` : Number of sensor_msgs/LaserScan topics you’ll use.

```num_multi_echo_laser_scans``` : Number of sensor_msgs/MultiEchoLaserScan topics you’ll use.

```num_point_clouds```: Number of sensor_msgs/PointCloud2 topics you’ll use.

```use_landmarks``` :  usage of landmarks

```use_nav_sat``` : usage of GPS

```use_odometry``` : to enable usage of odometry

```RAJECTORY_BUILDER_3D.num_accumulated_range_data```or ```TRAJECTORY_BUILDER_2D.num_accumulated_range_data```. This variable defines the number of messages required to construct a full scan 

```TRAJECTORY_BUILDER_nD.min_range``` and ```TRAJECTORY_BUILDER_nD.max_range``` : bandpass parameters

```TRAJECTORY_BUILDER_nD.num_accumulated_range_data```

```TRAJECTORY_BUILDER_nD.voxel_filter_size``` : reduce the incoming pointcloud

```TRAJECTORY_BUILDER_2D.use_imu_data```: use IMU, must in 3D

####Local SLAM:

Local SLAM inserts a new scan into its current submap construction by scan matching using an initial guess from the pose extrapolator

```TRAJECTORY_BUILDER_3D.ceres_scan_matcher...``` : parameters regarding ceres solver for optimizations 

```RealTimeCorrelativeScanMatcher```: another option for pose calc ( complex that only ```CeresScanMatcher```, search window )

```TRAJECTORY_BUILDER_nD.motion_filter...``` : motion filter options, drop scans if not significant enough

```TRAJECTORY_BUILDER_nD.submaps.num_range_data``` : amount of range data to define submap as complete

```TRAJECTORY_BUILDER_2D.submaps.range_data_inserter....``` grid probabilistic options

####GLobal SLAM:

arrange submaps between each other so that they form a coherent global map.

```POSE_GRAPH.optimize_every_n_nodes``` : optimization rate ( 0 disable)

```POSE_GRAPH.fast_correlative_scan_matcher...``` search window parameters

```POSE_GRAPH.constraint_builder.sampling_ratio``` : sampling ratio for constraints building

```POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.``` : exploration tree for the scan matching

#### Pure local

if we dont want to update the map (such as amcl)

```
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}
```

#### Multi-trajectories SLAM

This is achieved through the usage of two ROS services ```start_trajectory``` and ```finish_trajectory```

#### Writing state

Finish the first trajectory. No further data will be accepted on it.
```rosservice call /finish_trajectory 0```

Ask Cartographer to serialize its current state.
(press tab to quickly expand the parameter syntax)
```rosservice call /write_state "{filename: '${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream', include_unfinished_submaps: 'true'}"```

