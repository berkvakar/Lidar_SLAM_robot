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
├── my_robot_viz/             # live LiDAR viewer in the browser (Python)
│   ├── my_robot_viz/scan_web_server.py
│   ├── web/index.html
│   ├── launch/lidar_viz.launch.py
│   ├── package.xml
│   └── setup.py
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

## LiDAR viewer

`my_robot_viz` shows the live `/scan` in a web browser. The Pi serves the page,
so you can watch it from a laptop or phone on the same network.

One-time setup on the Pi (LD19 driver + serial port access):

```bash
cd ~/ros2_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
sudo usermod -aG dialout $USER   # then log out and back in
```

Build as above, then start the driver and the viewer together:

```bash
ros2 launch my_robot_viz lidar_viz.launch.py start_driver:=true
```

Open `http://<pi-ip>:8080` in a browser (find the IP with `hostname -I`).
The robot is in the centre, facing up. Scroll or pinch to zoom; the red circle
marks the closest obstacle.

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `start_driver` | `false` | Also start the LD19 driver. Leave `false` if `/scan` is already published. |
| `serial_port` | `/dev/ttyUSB0` | LD19 serial device (check with `ls /dev/ttyUSB*`). |
| `scan_topic` | `/scan` | Topic the driver publishes and the viewer shows. |
| `web_port` | `8080` | Port of the web page. |

The page has no password: anyone on the same network can view it.
