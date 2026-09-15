# optitrack_listener2

ROS 2 node that connects to an OptiTrack (Motive) server via the NatNet SDK
and publishes rigid body poses as `nav_msgs/Odometry`, with optional TF.

The package provides two executables:

- `mocap2_driver` — NatNet SDK based driver, supports streaming multiple
  rigid bodies at once, each published on its own topic.
- `optitrack_listener` — legacy node (see `launch/optitrack_listener.launch.py`).

## Dependencies

- ROS 2 (Humble)
- `px4_msgs`, `tf2`, `tf2_ros`, `rclcpp` and the other dependencies listed
  in `package.xml`
- NatNet SDK: headers and library are already bundled in the package under
  `NatNetSDK/include` and `NatNetSDK/lib/libNatNet.so` — no separate
  install needed. `CMakeLists.txt` automatically copies `libNatNet.so`
  next to the installed executable and sets `RPATH=$ORIGIN`, so there is
  no need to add it to `/usr/lib` or `LD_LIBRARY_PATH` manually.

## Build

```bash
cd ~/px4_ws
colcon build --packages-select optitrack_listener2
source install/setup.bash
```

## Run

```bash
ros2 launch optitrack_listener2 mocap2_driver.launch.py
```

## Default configuration

The parameters in [`launch/mocap2_driver.launch.py`](launch/mocap2_driver.launch.py)
are already set up as a working configuration for Unicast streaming from
Motive. For a normal setup you only need to change the two IP addresses:

```python
'connection_type': "Unicast",
'server_address': "172.31.1.150",   # <-- IP of the PC running Motive
'local_address': "172.31.1.104",    # <-- IP of this PC (the one running ROS)
'multicast_address': "239.255.42.99",
'server_command_port': 1510,
'server_data_port': 1511,

'odom_frame_id': "optitrack_link",
'map_frame_id': "optitrack_odom",
'publish_tf': False,

'rigid_body_ids': [0, 1, 2, 3],     # <-- edit with your rigid body IDs
```

Everything else (ports, frame ids, `publish_tf`) can normally be left as
is.

### On the Motive side

- In Motive's Streaming pane, set the streaming type to **Unicast**.
- Select, as the **local/network interface**, the network adapter on the
  Motive PC that actually talks to this PC (i.e. the one on the same
  subnet as `local_address` above). Streaming on the wrong interface is
  the most common reason for "not connected"/no data.

### Rigid body IDs

For every ID in `rigid_body_ids` a dedicated publisher is created:

```
/optitrack/body_<ID>/odometry
```

with `child_frame_id = <odom_frame_id>_body_<ID>`. IDs are looked up
frame-by-frame in the data received from NatNet (`RigidBodies[i].ID`); if
a requested ID is not present in the current frame, it is simply skipped
(no publish for that body in that frame).

To find the correct IDs, open Motive → the rigid body panel → "Streaming
ID" (a.k.a. "Asset ID").

## All parameters

| Parameter              | Type    | Default            | Description |
|-------------------------|---------|---------------------|-------------|
| `connection_type`       | string  | `"Unicast"`         | `Unicast` or `Multicast` |
| `server_address`        | string  | —                   | IP of the PC running Motive/NatNet server |
| `local_address`         | string  | —                   | IP of this machine (interface on the same network as the server) |
| `multicast_address`     | string  | `239.255.42.99`     | Used only if `connection_type: Multicast` |
| `server_command_port`   | uint16  | `1510`              | NatNet command port (Motive default) |
| `server_data_port`      | uint16  | `1511`              | NatNet data port (Motive default) |
| `odom_frame_id`         | string  | `"ot_odom"`         | Prefix used to build each body's `child_frame_id` (`<odom_frame_id>_body_<ID>`) |
| `map_frame_id`          | string  | `"map"`             | `frame_id` of the header of the published Odometry messages |
| `publish_tf`            | bool    | `true`              | If `true`, also publishes the `map_frame_id -> child_frame_id` TF for every body |
| `rigid_body_ids`        | int64[] | `[0]`               | List of rigid body IDs (Motive asset IDs) to publish |

## Troubleshooting

**`error while loading shared libraries: libNatNet.so: cannot open shared
object file`**
The target's RPATH is not set correctly, or the package hasn't been
rebuilt after updating `CMakeLists.txt`. Rebuild:
```bash
colcon build --packages-select optitrack_listener2
```
and check that `libNatNet.so` is present next to the executable:
```bash
ls install/optitrack_listener2/lib/optitrack_listener2/
```

**`... not connected :(`**
Check that Motive is actually streaming (Streaming Panel), that
`server_address` / `local_address` are correct, that Motive is streaming
on the network interface facing this PC, and that both hosts can reach
each other (`ping`).

**"No rigid body received in this frame" / no topic being published**
Check that the IDs in `rigid_body_ids` match the Streaming IDs configured
in Motive for the bodies you want to receive.
