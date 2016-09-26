# Trooper Plugin

This repository contains a [ROS](http://www.ros.org) plugin for the [KVH1750](https://github.com/jasedit/ros_kvh1750) node, which accumulates individual IMU messages into the RawCachedIMUData available as part of the [pronto](https://github.com/ipab-slmc/pronto-distro) package, and required for getting that package to work on robots which have a KVH 1750, but are not an ATLAS.

# Installation

1. Make sure `trooper_mlc_msgs` is part of the installed/built ROS packages for dependency resolution. One method of doing this is cloning [pronto](https://github.com/ipab-slmc/pronto-distro), then symlinking the trooper_mlc_msgs package from `pronto-lcm-ros-translators/src/trooper_mlc_msgs` into the catkin workspace.
2. Clone this repository into the src directory of a catkin workspace.
3. Execute `catkin_make install` in the catkin root directory (e.g. `~/catkin_ws`, not `~/catkin_ws/src`)
4. The plugin is correctly installed ROS if the command `rospack plugins --attrib=plugin kvh1750_trooper_plugin` returns text listing the location of the plugin.

# Usage

When launching the ros_kvh1750 node, it will load the plugin defined in the node's private namespace under `processor_type` (e.g. `/ros_kvh1750/processor_type`). Setting that value to `kvh1750_trooper_plugin` will load the plugin for operation.