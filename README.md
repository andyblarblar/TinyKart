# TinyKart

TinyKart is an autonomous RC car.

## Concept

TinyKart is currently a fairly standard mobile robot, simply using nav2 to navigate. This is not great for high speeds, so we plan on migrating to an end to end machine learning solution in the future. Currently, the only odometry source is the IMU, which we have fed into robot_localisation. Robot_localisation has its control mode on, so in theory the cmd_vel inputs from nav should be assumed to be the odom, and the IMU provides some correction. In the case this causes too much error, we are looking at using VIO as a backup odom source. Waypoints are provided in a csv file, and are followed after pressing the start button on a [wii remote](https://github.com/andyblarblar/ros2_wii_wheel), which is our teleop controller of choice. TinyKart does use DWB, so it should avoid obsticles dynamically.

## Building

1. clone the repo
2. `vcs import` the tinykart.repos file
3. rosdep dependencies
4. pip install the dependancies in tkio_ros
5. `colcon build` the workspace

## Running

Launch sim with: `ros2 launch tinykart_gazebo tinykart.launch.py`

Launch IRL with: `ros2 launch tinykart_robot tinykart.launch.py`

## Hardware

TinyKart utilises the follwing hardware setup:
- 2wd stock Traxass Slash body
- ldrobot LD06 Lidar attached over a tty converter to the PC
- bno055 IMU in uart mode connected to a tty converter to the PC
- rp2040 board running [tkio](https://github.com/andyblarblar/tkio-pico). Connect the uart pins to a tty converter to the PC, and the remaining pins to the slash as described in it's readme
- Computer with bluetooth onboard

When setting up the car physically, first plug everything in. Next, turn on the cars ESC. Now plug in the pico, which needs to be connected to the ESC when it boots, as that's when it arms the ESC. Now just start ROS on the PC, and it should boot right up. Currently, the wiimote is not booted with the repo, so start that prior if that is your source of teleop. You should now be able to teleop, and press the home button to start auton.

We will open source our schematics for the chassis and electronics after we more throughly test them.

# Troubleshooting

- Ign Gazebo currently does not provide TF for ackermann drive robots on the binary releases. I've sucessfully merged a PR that fixes this, but as of now (8/3/22), you must build gazebo off master for sim to work correctly. Gazebo 6.11 should fix this.
- If sim simply does not give scans, make sure that sensor_processor is on its gazebo_sim branch, as only that branch does the remappings we need for sim.
