


-> copy the function definition and the function call over into this file
-> replace doControl


# General
## ToDo
* Different authors of the files?

## Done
* Added header in each cpp and h file (adapted from https://github.com/youbot/youbot_driver_ros_interface/blob/indigo-devel/src/YouBotOODLWrapper.cpp )

## To be checked:
* check if control weight is set to 0 when wheel fails
* do we need all parameters in platformdriver.h
* check wheeldata struct what is necessary for interacting with ROS

## Open issues
* Add an example for interfacing with the controller. -> joypad controller (ROS interface)
* Add an example for interfacing without ROS
* use timestamp from wheelmeasurements to determine the exact timing of the controlloop
* plot all parameters nicely to screen during init and dump them into a log file
* implement monitors:
  * check for wheel is not working anymore (e.g. monitor that the ts is actually changing)
  * every so often check if variance between different wheels is not too large
  * if no command is received or ethercat com goes down -> emergency stop
* check with Arthur: is the assumption that workcount during init is the correct one or could some slaves join late. (platfromdriver.cpp 405)
* make the loop speed configurable
* platformdriverros.cpp clean some of the commented stuff up
* velocitycontroller.cpp -> hardcoding of wheel positions should come from configfile (line 30 ff)

# config files
## ToDo
* replace wheels with yaml list
* also remove the reverse velocity param -> then remove hardcoded values from out testing and replace with local default params
* add wheel diameter and dist 
* remove master and replace with ethercat config

## Done
* delete unnecessary examples
* clean the content of the config files (check which params are still in code)


# KeloDriveAPI.h
* removed senseparam
* removed drvparam
* removed param
* removed gripper
-> yes, remove unused structs. otherwise we should also provide an example for how to use it
robile is about simplicity, so do not present it, but have them in an "advanced manual" e.g. provided with KELO drive but not with robile

# Structs.h
## Done
* removed inertia
* removed point3d
