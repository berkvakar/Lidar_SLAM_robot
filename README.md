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
├── my_robot_viz/             # browser test tools (Python): LiDAR viewer + odometry test bench
│   ├── my_robot_viz/scan_web_server.py
│   ├── my_robot_viz/odom_test_server.py
│   ├── web/index.html        # LiDAR page
│   ├── web/odom.html         # odometry page
│   ├── launch/lidar_viz.launch.py
│   ├── launch/robot_test.launch.py   # pick test:=lidar|odom|both
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

## Choosing a test tool: LiDAR or odometry

`robot_test.launch.py` starts one or both browser test tools:

```bash
ros2 launch my_robot_viz robot_test.launch.py test:=lidar   # LiDAR viewer only     (port 8080)
ros2 launch my_robot_viz robot_test.launch.py test:=odom    # odometry test only    (port 8081)
ros2 launch my_robot_viz robot_test.launch.py test:=both    # both (default)
```

Each tool has its own page and port, so `both` gives you two tabs. The Pi is
headless, so reach the pages through an SSH tunnel from the laptop (PowerShell):

```powershell
ssh -L 8080:127.0.0.1:8080 -L 8081:127.0.0.1:8081 <user>@<pi-ip>
```

Then open `http://127.0.0.1:8080` (LiDAR) and `http://127.0.0.1:8081` (odometry).
All LiDAR arguments from the table above work here too; `web_port` is called
`lidar_port` in this launch file.

## Odometry test bench

The odometry page watches `nav_msgs/Odometry` on `/odom` and can run test
moves by publishing `geometry_msgs/Twist` on `/cmd_vel`:

| Test | Amount | Speed |
|---|---|---|
| Drive straight | metres (negative = backwards) | m/s |
| Rotate in place | degrees (positive = left/CCW) | °/s |
| Spin wheels | wheel revolutions, both forward or opposite | rev/s |

"Stop when" picks how a move ends:
- **odometry reaches target** (closed loop): the robot stops when `/odom` says
  it has gone far enough. Use this for calibration.
- **time is up** (open loop): the robot runs for distance ÷ speed seconds.
  The error column then shows how well the motors follow the command.

The page shows the driven path (odom frame, start direction up), live x / y /
yaw / v / ω, a commanded-vs-measured speed chart, and a results table.
**Export CSV** saves every sample.

Arguments (in addition to `test`):

| Argument | Default | Meaning |
|---|---|---|
| `allow_motion` | `false` | Must be `true` for tests to drive the robot. Off = view only. |
| `fake_odom` | `false` | Simulate `/odom` from the commands. Try the page without a motor driver. Don't use it while a real driver publishes `/odom`. |
| `odom_topic` | `/odom` | Odometry topic. |
| `cmd_vel_topic` | `/cmd_vel` | Velocity command topic. |
| `wheel_radius` | `0.0325` | Wheel radius in m (65 mm TT wheel); used to turn revolutions into distance. |
| `wheel_separation` | `0.14` | Left-right wheel distance in m. |
| `odom_port` | `8081` | Port of the odometry page. |

Safety: speeds are capped (0.3 m/s, 1.5 rad/s), each move is limited to 30 s,
and the robot stops on **STOP** / Esc, when odometry stops arriving, or when the
last page tab closes. Anyone on the network who can reach the page can drive
the robot while `allow_motion:=true`, so keep it off when you're not testing.

Try it without hardware:

```bash
ros2 launch my_robot_viz robot_test.launch.py test:=odom fake_odom:=true
```

### Calibrating odometry

The odometry driver (a later phase) turns encoder ticks into `/odom` using its
own `wheel_radius` and `wheel_separation`. To calibrate them:

1. Run **Drive straight**, 1 m, stop on odometry. Measure the real distance
   with a tape and type it in **Real**. Copy the suggested `wheel_radius`
   into the driver. Repeat until the error is small.
2. Then run **Rotate in place**, 360°. Judge the real angle (e.g. a tape line
   on the floor and on the robot), and type it in. Copy the suggested
   `wheel_separation`.

On a 4-wheel skid-steer chassis, the calibrated `wheel_separation` is usually
noticeably larger than the measured distance between the wheels. That is
normal: the wheels slip sideways while turning.
