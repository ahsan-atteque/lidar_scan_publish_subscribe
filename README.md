# rplidar_s2_driver

Pure-Python ROS 2 driver for the SLAMTEC RPLIDAR S2. Publishes
`sensor_msgs/LaserScan`.

## Why this exists

The Debian-packaged `rplidar_ros` (SDK 1.12.0, Jazzy/noble arm64) connects to
the S2 successfully — reads serial number, firmware 1.01, health 0 — and then
segfaults the instant it starts scanning:

```
#0  __pthread_mutex_unlock_usercnt (mutex=0x1, decr=1)
#1  rp::standalone::rplidar::RPlidarDriverImplCommon::startMotor()
#2  rplidar_ros::rplidar_node::start()
```

`mutex=0x1` is not a valid pointer. That code path is the legacy
`rp::standalone::rplidar` API which predates S2 support; the S2 is meant to be
driven through SLAMTEC's newer `sl::` API. Rather than patch and rebuild a
vendor SDK on-device, this implements the wire protocol directly.

## Install

```bash
mkdir -p ~/ros_ws/src
cp -r rplidar_s2_driver ~/ros_ws/src/

cd ~/ros_ws
apt install -y python3-serial          # or: pip3 install pyserial
colcon build --symlink-install --packages-select rplidar_s2_driver
source install/setup.bash
```

## Run

```bash
ros2 launch rplidar_s2_driver s2_launch.py
```

Bench testing with no robot description loaded — RViz needs a TF entry for
`laser`, and with no URDF nothing provides one:

```bash
ros2 launch rplidar_s2_driver s2_launch.py use_static_tf:=true
```

Or the node directly:

```bash
ros2 run rplidar_s2_driver s2_node --ros-args \
  --params-file src/rplidar_s2_driver/config/s2_params.yaml
```

## Verify

```bash
ros2 topic hz /scan          # expect ~10 Hz, std dev under 1 ms
ros2 topic echo /scan --field ranges --once
```

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `serial_port` | `/dev/rplidar` | Prefer the udev symlink; `ttyUSB*` numbering shifts between replugs |
| `serial_baudrate` | `1000000` | **S2 is 1 Mbaud.** A1 is 115200, A2 is 256000 |
| `frame_id` | `laser` | Must match your URDF's laser link |
| `topic` | `scan` | |
| `range_min` / `range_max` | `0.05` / `30.0` | S2 spec. Using the A1's 12 m discards valid long returns |
| `inverted` | `false` | Set true if mounted upside down |
| `angle_compensate` | `true` | Bin into uniform angular array — LaserScan requires constant spacing, the S2 doesn't provide it |
| `samples_per_scan` | `1600` | Bin count. ~1600 samples/rev at 10 Hz |
| `min_quality` | `0` | Raise to ~10 to reject phantom returns off reflective surfaces |

## Design notes

**Two threads.** Serial reads block and a revolution takes ~100 ms. Doing that
inside an rclpy timer callback would stall the executor for the full duration,
delaying parameter services and making Ctrl+C sluggish. The reader thread owns
the serial port; the executor owns ROS.

**One-slot queue, drop-oldest.** For live sensor data a stale scan is
worthless. If the publisher falls behind, skip ahead rather than build an
unbounded backlog of increasingly wrong data.

**One-byte resync.** On a framing error the parser discards exactly one byte,
not five. Nodes are 5 bytes wide but corruption can start at any offset —
dropping 5 preserves misalignment forever, dropping 1 walks back into phase
within at most 4 iterations. This is the detail most implementations get
wrong.

**Motor via DTR.** The CP2102 adapter gates the motor brake on DTR, and it's
inverted: DTR **low** releases it. Driving the line directly sidesteps the
`startMotor()` crash entirely.

**BEST_EFFORT QoS.** Matches how standard LiDAR drivers publish and how RViz
subscribes by default. RELIABLE would make RViz silently fail to match with no
error shown.

**Clean shutdown.** `destroy_node()` stops the motor and the scan. Without it
the rotor keeps spinning after Ctrl+C, and the device is left mid-stream so the
*next* session's `GET_INFO` gets swallowed by leftover data — which looks
exactly like dead hardware.
