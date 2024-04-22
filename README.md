# Dingo Quadruped

<p align="center">
    <img src="assets/JEL05566.jpg" style="align:centre" width="50%">
</p>

# STUFF FOR PROJECT
Welcome to our repo! All of our code is in master so you should be able to run from master, just follow the steps below!

1. Follow the `Installation of Code`

To run our experiment in the intended way, use the commands

## Installation of Code
### Natively
- Install Ubuntu 20.04
- Install [ros-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Install necessary packages via `sudo apt-get install python3-catkin-tools git python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-pip build-essential wireless-tools ros-noetic-joy ros-noetic-catkin python3-catkin-tools i2c-tools libi2c-dev python3-smbus`
- Install necessary python packages via `pip install spidev transforms3d adafruit-circuitpython-bno055 pillow rpi.gpio pyserial`
- Change directory to the home folder: `cd ~`
- Clone this (the Dingo Quadruped) repository using git: `git clone ...`
- Move into the dingo_ws folder: `cd /DingoQuadruped/dingo_ws`
- Initialise rosdep: `sudo rosdep init`
- Fetch dependencies with rosdep: `rosdep update`
- Build the workspace: `catkin build`
- Source the workspace: `source devel/setup.bash`
- (Optional) Add a line to .bashrc to automatically source the workspace: `echo "source ~/DingoQuadruped/dingo_ws/devel/setup.bash" >> ~/.bashrc`, `source ~/.bashrc`

## Running the code
### Dingo_Driver
The Dingo_Driver should be started before any other code is launched on the Dingo. It starts joystick control of the robot and allows joint and task space commands to be received from other code or controllers via command ROS topics, as long as joystick control is disabled. If enabled, joystick control will override any commands sent through the command topics. To launch it, run the following line:
`roslaunch dingo dingo.launch`

Arguments are:
- is_physical (0/1): Is the code being run on the Dingo itself? Default: "1" (Yes)
- is_sim (0/1): Should the code publish joint values to the simulator? Default: "0" (No)
- use_joystick (0/1): Is a joystick being used for control? Default: "1" (Yes)
- use_keyboard (0/1): Is the keyboard being used for control? Default: "0" (No)
- (currently not used) serial_port (name of port): The serial port that the nano is connected to. Default: "/dev/ttyS0"
- use_imu (0/1): Should IMU data be used to correct the robots joint angles? Default: "0" (No)

With no arguments specified, it will assume a joystick controller is used for control and it will launch the hardware interface with IMU feedback disabled. No joint data will be published for Gazebo to use to simulate the robot.

As an example of how the arguments can be used, if the code is to be run purely in simulation with joystick control, you would launch the driver with the following arguments: 
`roslaunch dingo dingo.launch is_physical:=0 is_sim:=1`

### Dingo Joystick Controls
<p align="center">
    <img src="assets/joystick control map.jpg" style="align:centre" width="50%">
</p>

The diagram above shows the mapping of joystick commands for the Dingo. These instructions are based on a PS4 type controller. An alternative, more general description of joystick commands is below:

The Dingo has two main states: Rest and Trot. 
- While in the TROT state: Left stick controls the robot's movement. Right stick controls the robot's tilt and turning.
- While in the REST state: Left stick is disabled. Right stick controls the robot's gaze while staying in place.

Buttons:
- Joystick Control Toggle (L1 Button): Pause/Resume control from the joystick.
- Gait Toggle (R1 Button): Toggles between trotting and resting modes.
- Hop Request (X button): Perform a hop (Currently not implemented).

Movement: (Speed proportional to how far stick is moved)
- Left Stick (Horizontal): Pushing left or right strafes the robot in that direction.
- Left Stick (Vertical): Push up to move forward, down to move backward.

Gaze:
- Right Stick (Horizontal): Pushing left or right turns the robot in that direction.
- Right Stick (Vertical): Push up to raise front of robot, down to raise back of robot.

Orientation:
- D-pad (Vertical): Pressing up or down adjusts the height of the robot.
- D-pad (Horizontal): Pressing left or right tilts the robot in that direction.

### Launching the gazebo simulation
Make sure dingo_driver is running first, then:
`roslaunch dingo_gazebo simulation.launch`


### Overview
This repository hosts the code for the Dingo Quadruped, a robot designed to be low-cost but capable of conducting research and being extensively modified with additional actuators and sensors. CAD for the Dingo can be found [here](https://grabcad.com/library/dingo-robot-quadruped-2). A full Bill of materials for purchasable components can be found within this repo.

This code is based on the [Stanford Pupper](https://github.com/stanfordroboticsclub/StanfordQuadruped) and [notspot](https://github.com/lnotspotl/notspot_sim_py) codebases, with extensive modifications, including integration into ROS 1 Noetic.

The repository includes a driver node, dingo_driver.py, which should be used anytime the code is run. This file enables joystick control of the Dingo, and allows joint and/or task space commands generated by any other code to be passed through the driver via the appropriate ROS command topics. The joystick can also be toggled on and off to override commands received via ROS command topics

The repo also includes a gazebo simulation of the Dingo, based on URDF file and meshes which are also provide.

### How Dingo_driver Works
The following flow diagram shows a simplified overview of how a joystick command is handled by the driver to affect joint movements:
<p align="center">
    <img src="assets/Dingo_driver flow diagram.png" style="align:centre" width="50%">
</p>

### Project Structure
```.
├── assets                                    Images used in the readme file
├── dingo_nano                                Code for the Arduino Nano V3 to read sensor data and send it to the Raspberry Pi
└── dingo_ws                                  ROS workspace containing all required packages
   └── src
     ├── dingo                                Package containing node and launch files for running the robot
     ├── dingo_control                        Package containing all files related to control, including kinematics and default trot controller
     ├── dingo_description                    Package containing simulation files (URDF file and meshes)
     ├── dingo_gazebo                         Package containing gazebo files
     ├── dingo_hardware_interfacing
     |  ├── dingo_input_interfacing           Package containing files for receiving and interpreting commands (From a joystick or keyboard)
     |  ├── dingo_peripheral_interfacing      Package containing files for interfacing with the Arduino Nano, LCD screen and IMU
     |  └── dingo_servo_interfacing           Package containing the hardware interface for sending joint angles to the servo motors
     └── dingo_utilities                      Package containing useful utilities
```


