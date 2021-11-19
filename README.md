# KELO Tulip

This package contains the *KELO Tulip* software. This software takes a velocity vector for the overall platform and converts it to commands for the individual KELO Drives of the platform. It implements an EtherCAT master to communicate with the KELO Drives and provides a simple velocity controller that can be used on real robots as well as for simulation.

## KELO Drives and how to build a platform
The [KELO Drive](https://www.kelo-robotics.com/technologies/#kelo-drives) is the patent pending novel drive concept for mobile robots developed by KELO robotics. These wheels can be attached to any rigid platform in order to transform it into a mobile robot. It can even be used to [robotize a container](https://www.kelo-robotics.com/customized-designs/#robotized-material-container). A few screws are enough.

After mechanically connecting the KELO Drives to your platform, you only need to connect them to power via an XT-60 power connector and to your computer via an EtherCAT cable. For multiple wheels, you should either use a Beckhoff Ethernet Switch or one of the KELO power distribution boards. After the wheels are powered and connected to your computer via EtherCAT, the software in this repository will help you make your robot move. 

You can move your mobile platform via a joypad for test purposes or use any software that publishes ROS `geometry_msgs/Twist` messages to the `cmd_vel` topic. See the Interface section below. The software can also be used without ROS. Please contact the developers for that.

## System requirements

This software was tested on Ubuntu 16 with ROS Kinetic, Ubuntu 18 with ROS Melodic and Ubuntu 20 with ROS Noetic. Other Linux flavors should work as well.

For ROS it is enough to install the base system (for Noetic ros-noetic-ros-base). In addition it requires

```
sudo apt install ros-noetic-tf
```

If another ROS version is used, replace the term noetic accordingly in these commands.

## Installation

The package can be compiled like any ROS package. Clone or copy it into a ROS workspace source folder and run `catkin_make` or `catkin build`, depending on your preferences.


### Permissions

Special permissions need to be granted to the generated executable, since it needs *RAW* access to the Ethernet port. Instead of starting it as root the `setcap` tool can be used:
 
```
sudo setcap cap_net_raw+ep <name_of_executable>
```

Optionally, the command can be applied during the build process by passing the `-DUSE_SETCAP=ON` option to catkin. Default is `OFF`.

```
catkin_make -DUSE_SETCAP=ON
```
OR
```
catkin build kelo_tulip -DUSE_SETCAP=ON
```

**Note**: If you use this flag, you will be asked to input the sudo password during the build process. It might look like the build is going on but you need to lookout for `[sudo] password for <username>` in the output and enter the password after this prompt had appeared. If not, the build process will continue forever.

### Finding dynamic libraries

If using Ubuntu 18 or newer (with ROS Melodic or Noetic), running the setcap command as described above can cause the executable to not find all dynamic libraries anymore. This is because the dynamic linker works in a "secure-execution mode" when the capabilities of a program changed, in which it ignores most environment variables such as LD_LIBRARY_PATH. A typical error after starting reads like this:

```
devel/lib/kelo_tulip/platform_driver: error while loading shared libraries: libtf2_ros.so: cannot open shared object file: No such file or directory
```


In that case it is recommended to set the path to those libraries on system level. The following method should work on default installations, but please adapt accordingly if you already made other changes in your system.

For ROS Noetic, the following command will add an entry to point to the dynamic libraries of ROS:

```
sudo sh -c 'echo "/opt/ros/noetic/lib/" > /etc/ld.so.conf.d/ros.conf'
```

If not using ROS Noetic, the path there should be changed accordingly.

Afterwards you need to run

```
sudo ldconfig
```

to make the changes take effect.

## Usage
### Starting the program

The program can be started by running

```
roslaunch kelo_tulip example.launch
```

### Parameters

The default launch file in `kelo_tulip/launch/example.launch` loads the YAML configuration from `config/example.yaml`. Feel free to change parameters directly in this config file, or to make a copy and adjust the launch file to load the new file.

#### Network interface

The following setting defines the network interface used by the driver:

```
device: enp2s0
```

This setting must be adjusted to the network interface by which the KELO drives are connected via EtherCAT.

#### Wheels

The controller needs to know the number of wheels and their location in the body fixed frame of the platform as well as the offset of their pivot encoder (the encoder value when the wheel is oriented forward). This information should be included in the YAML configuration file in the following manner:

```
num_wheels: 4

wheel0:
  ethercat_number: 6
  x: 0.175
  y: 0.1605
  a: 3.14

wheel1:
  ...
```

Please adjust the list of wheels with the correct number and location of the wheels on your platform. `wheel0` refers to the first wheel, starting counting with zero. The `ethercat_number` is the EtherCAT slave number of that wheel. The possible numbers can be seen from when starting kelo_tulip, it will print out information about all slaves found on the EtherCAT bus.

The values `x` and `y` are the coordinates of the wheels center according to the fixed frame of the platform in meters. `a` is the offset of pivot encoder in rad. 

### ROS Interfaces

Currently the kelo_tulip software uses ROS as a middleware, subscribing resp. publishing to the following topics.

#### /cmd_vel

This topic accepts [`geometry_msgs/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) messages. Any motion software that creates a velocity vector for the platform and publishes `geometry_msgs/Twist` messages to the `cmd_vel` topic can be used. The ROS package [`move_base`](http://wiki.ros.org/move_base) is an example that conforms to that. 


#### /joy

By sending messages to the `/joy` topic the platform can be moved by a joypad. The [`joy` ROS package](http://wiki.ros.org/joy) can be used to send these messages via joypad. In the function `joyCallback()` in `PlatformDriverROS.cpp` is a simple translation between joystick input and platform velocity command.

Note: For safety reasons joystick messages are only considered if the `RB` button on the joypad is pressed, resp. the joy button with index 5 is active. This makes it also possible to run it in parallel with other packages that already process joystick input, overriding their input as long as the `RB` button keeps being pressed.

#### /odom and /tf

On the topic `/odom` odometry data in form of [`nav_msgs/Odometry`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) are published. Each time the program is started, the position is reset to the origin.

The same odometry data is published also on the topic `/tf` in the form of tf transform from the frame `base_link` to `odom`.

#### /status

On this topic an integer representing status information about the controller is published periodically in form of [`std_msgs::Int32`](https://docs.ros.org/en/api/std_msgs/html/msg/Int32.html) messages. The single bits of the number have the following meaning:

| Bit       | Description                                                                                                                             |
|-----------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 0x0001    | Status OK, EtherCAT communication and all drives are working properly.
| 0x0100    | An unspecified error occured.
| 0x0200    | Error detected in EtherCAT communcation because of wrong WKC value. Might be temporary communication loss or power off of one wheel.
| 0x0400    | Timestamp of one KELO Drive did not increase as expected, probably it stopped communicating.
| 0x0800    | One KELO Drive did not have the expected status bits set, probably it got deactivated for some reasons.


### Velocity controller

kelo_tulip includes a simple velocity controller to demonstrate the general principle how to command wheel velocities. It considers velocity and acceleration limits for the platform. Being velocity based it does not allow for the compliant motion of the platform. It is rather meant as a guideline how own controllers can be developed. The controller itself is implemented as a C++ class and does not have any dependencies on ROS. 

The concept of this simple controller is as follows. For each drive a target velocity is computed that depends on the desired velocity for the whole platform and the mounting position of the drive on the platform. The velocity of the individual wheel determines which pivot angle it should have. This pivot angle must be achieved by rotating the wheel around its center. This can be done by moving the left and right hubwheel with the same speed in opposite directions; similar to like a robot with differential drive would rotate around its center. In addition the linear motion of the drive can be achieved by giving the left and right hubwheel the same speed in the same direction. Just overlaying both parts, speeds in opposite direction for pivot-rotation and speeds in same direction for linear motion, will lead to the desired overall motion of the platform.

The crucial part here is that each drive is handled on its own, separately from the others. This makes the whole approach very modular and does not cause any issues when more wheels are added or their mounting locations are changed. A flaw of this simple approach is the tendency to give already a forward motion if the wheels do not point yet into the correct orientation. If different wheels then start to move into different directions, the result on the platform can be very suboptimal. One possible solution is to delay giving forward velocities until all wheels are oriented more or less correctly. This can increase the stability of the motion, but also make maneuvers slower, and for the sake of simplicitiy was not done in the provided example controller.

The controller can be found in the file `src/VelocityPlatformController.cpp`. The main functions are the following:

#### VelocityPlatformController()

The constructor initializes several variables, there are in particular:

- `platform_target_vel_`: A struct with `x`, `y`, and `a` members to set the target values as requested by the external application
- `platform_ramped_vel_`: A struct with `x`, `y`, and `a` members to set velocities that acceleration limits into account
- `platform_limits_`: A struct with minimum and maximal settings for velocity, acceleration and deceleration

#### setPlatformTargetVelocity()

Called to set the desired platform velocity in x, y, and rotational direction.

#### initialise()

This function is called with a configuration setting for each wheel, in particular containing the location of each wheel in the platform's coordinate center.

#### calculatePlatformRampedVelocities()

This function ramps up or down the current velocity setpoints according to the acceleration limits and the platform's target velocities. Each dimension is considered separately from the others.


#### calculateWheelTargetVelocity()

This is the main function that computes the setpoint for the left and right hub wheels of each drive. It must be called once for each drive at each step. The computation consists of the following main steps:

1. Determine the x and y position of each (left and right) hub wheel relative to the platform's center.
2. Calculate the velocity this wheel unit should have at its pivot position, i.e. between the left and right hub wheel.
3. Calculate the target pivot angle based on the wheel velocity.
4. Apply a simple P-controller to minimize the error between measured pivot angle and target pivot angle.
5. Calculate the single hubwheel velocity based on the pivot-controller result and the target velcoity of the drive.

The result is the setpoints for the left and right hubwheel, which can then send to the real KELO drive.
















