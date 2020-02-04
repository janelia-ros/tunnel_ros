Tunnel ROS Interface
======================

This is the ROS tunnel interface.

Published Topics
----------------
* `/tunnel_state` (`smart_cage_msgs/TunnelState`) - A joint state message containing the current state of all joints.

Subscribed Topics
-----------------
* `/tunnel_joint_target` (`sensor_msgs/JointState`) - Set joint state targets.

Parameters
----------
* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.

Command Line Examples
---------------------

```bash
ros2 launch tunnel tunnel.launch.py
ros2 launch tunnel tunnel_and_data_writer.launch.py
ros2 topic echo /tunnel_state
ros2 topic pub -1 /tunnel_joint_target sensor_msgs/JointState "{name: [right,left], position: [1000,1000]}"
```
