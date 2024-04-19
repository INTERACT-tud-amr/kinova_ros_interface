# kinova_ros_interface

## Requirements
1. Install Kinova python-API:
https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md

## Running a real robot
``` bash
    roslaunch kinova_ros_interface kinova_driver.launch
```

The default control mode is `High Level Velocity`. If you want to change the control modes, you need to call rosservices:
1. `/kinova/change_to_HLC_position` - The mode is switched to HLC position control and the `/kinova/command` topic expects desired joint positions
2. `/kinova/change_to_HLC_velocity` - The mode is switched to HLC position control and the `/kinova/command` topic expects desired joint velocities
3. `/kinova/change_to_LLC_position` - The mode is switched to LLC position control and the `/kinova/command` topic expects desired joint positions
4. `/kinova/change_to_LLC_velocity` - The mode is switched to LLC position control and the `/kinova/command` topic expects desired joint velocities. `NOT RECOMMENDED YET`

Topics:
1. `/kinova/command` - Topic for controlling kinova's joints
2. `/kinova/error_ack` - Topic for acknowledging faults
3. `/kinova/gripper` - Topic for controlling relative fingers' positions
4. `/kinova/joint_states` - Topic for subscribing the actual state of the robot in [rad, rad/s, Nm]






