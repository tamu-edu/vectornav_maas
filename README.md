# Vectornav ROS2 Driver

A ROS2 node for VectorNav INS / GNSS devices. 

This package that provides both raw and sensor_msg interfaces for the VN100, 200, & 300 devices. 
It has been entirely redesigned from the ROS1 package to provide a good basis to build into applications
without requiring modification of the node itself. The majority of the device configuration settings are 
exposed as ROS2 parameters that can be modified from a launch file. 


## QuickStart

Build

1. git clone https://github.com/dawonn/vectornav.git -b ros2
2. cd vectornav 
3. colcon build

Run with ros2 run (Option 1)

4. (Terminal 1) ros2 run vectornav vectornav
5. (Terminal 2) ros2 topic echo /vectornav/raw/common
6. (Terminal 3) ros2 run vectornav vn_sensor_msgs
7. (Terminal 4) ros2 topic echo /vectornav/imu

Run with ros2 launch (Option 2, uses parameters from `vectornav.yaml`)

8. (Terminal 1) ros2 launch vectornav vectornav.launch.py
9. (Terminal 2) ros2 topic echo /vectornav/raw/common
10. (Terminal 3) ros2 topic echo /vectornav/imu

## vectornav node

This node provides a ROS2 interface for a vectornav device. It can be configured
via ROS parameters and publishes sensor data via custom ROS topics as close to raw as possible.


## vn_sensor_msgs node

This node will convert the custom raw data topics into ROS2 sensor_msgs topics to make it easier 
to integrate with other ROS2 packages. 


## vn_set_antenna_config utility

A standalone utility to set or dump the GPS Antenna Offset and GPS Compass Baseline
registers on a VectorNav sensor from ROS2 parameters. Antenna offset is available on
GPS-capable devices (VN-200/VN-300/VN-310); the compass baseline is a dual-antenna
feature (VN-300/VN-310).

### Dump current register values (read-only)

```bash
ros2 launch vectornav set_antenna_offset.launch.py
# or:
ros2 run vectornav vn_set_antenna_config \
    --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baud:=115200 -p read_only:=true
```

### Write register values

Edit `config/antenna_offset.yaml` (set `read_only: false`, fill in `gps_antenna_offset`
and/or `gps_compass_baseline`), then:

```bash
ros2 launch vectornav set_antenna_offset.launch.py
```

The utility first prints the current register values (so you can recover them if a
write goes wrong), then writes the new values and reads them back for verification.

To persist the written values to flash so they survive power cycle, set `persist: true`:

```bash
ros2 run vectornav vn_set_antenna_config \
    --ros-args -p serial_port:=/dev/ttyUSB0 -p read_only:=false -p persist:=true
```

Note: `writeSettings()` saves **all** current register values to flash, not just the
ones written by this utility. If `read_only: true` and `persist: true` are both set,
`writeSettings()` is skipped with a warning.

### Export a dump to a params file

The dump output includes ROS log prefixes and read-only registers (estimated baseline)
that make copy-pasting difficult. Use the export script to produce a clean YAML:

```bash
ros2 run vectornav vn_set_antenna_config \
    --ros-args -p serial_port:=/dev/ttyUSB0 -p read_only:=true \
  2>&1 | python3 scripts/dump_to_config.py > my_antenna_offset.yaml
```

ROS2 logs go to stderr, so `2>&1` is required before the pipe. The generated
file is a drop-in replacement for `config/antenna_offset.yaml`. Only writable
registers are exported (the read-only estimated baseline is omitted).

### Volatile writes (important)

Register writes via this utility are **volatile** — values revert to whatever is stored
in the sensor's flash on power cycle. To persist values, either:

1. Set `persist: true` to call `writeSettings()` on the sensor (saves all current
   register values to flash), or
2. Re-run this config on each boot (e.g. add the launch file to your robot startup).


## References

[1] [VectorNav](http://www.vectornav.com/)
