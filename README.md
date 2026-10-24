# ROS 2 LiDAR robot

ROS 2 software for a Raspberry Pi robot with an LD19 LiDAR on a 4WD TT-motor chassis.
Target distro: Humble or Jazzy.

## Layout

This is a multi-package repo. Each top-level folder is one ROS 2 package.

```
.
├── my_robot_slam/            # slam_toolbox config + launch (online async mapping)
│   ├── config/mapper_params_online_async.yaml
│   ├── launch/slam.launch.py
│   ├── maps/                 # saved maps go here
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md
```

## Build and run (on the Pi)

Clone this repo into `~/ros2_ws/src/`, then:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch my_robot_slam slam.launch.py
```

SLAM waits for a `/scan` topic (LiDAR driver) and an `odom -> base_footprint` TF
(odometry). Neither exists yet; both come in later phases, so for now the SLAM
node starts but produces no map.
