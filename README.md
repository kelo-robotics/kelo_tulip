# KELO Tulip

This package contains the *KELO Tulip* software. This software takes a velocity vector for the overall platform and converts it to commands for the individual KELO Drives of the platform. 

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

**Note**: If you use this flag, you will be asked to input sudo password during
the build process. It would look like the build is going on but you need to keep
a lookout for `[sudo] password for <username>` in the output and enter the
password as soon as this prompt is visible. If not, the build process will
continue forever.

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

The controller needs to know the location of the wheels in the body fixed frame of the platform as well as the offset of their pivot encoder (the encoder value when the wheel is oriented forward). This information should be included in the YAML configuration file in the following manner:

```
wheel0:
  ethercat_number: 6
  x: 0.175
  y: 0.1605
  a: 3.14
  reverse_velocity: 1
```

Please adjust the list of wheels with the correct number and location of the wheels on your platform. `wheel0` refers to the first wheel, starting counting with zero. The `ethercat_number` is the EtherCAT slave number of that wheel. `x` and `y` are the coordinates of the wheels center according to the fixed frame of the platform. `a` is the offset of pivot encoder in rad. The flag `reverse_velocity` should be set to 1 if the wheels are mounted in a way that giving a forward velocity would drive the robot backwards.


### Interface

Currently the software uses ROS as a middleware and is subscribed to the `cmd_vel` topic. This topic accepts [`geometry_msgs/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) messages. Any motion software that creates a velocity vector for the platform and publishes `geometry_msgs/Twist` messages to the `cmd_vel` topic can be used. The ROS package [`move_base`](http://wiki.ros.org/move_base) is an example that conforms to that. 

The platform can also be moved by a joypad. We recommend using the [`joy` ROS package](http://wiki.ros.org/joy).
