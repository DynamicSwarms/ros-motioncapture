# Info
Copied from https://github.com/IMRCLab/motion_capture_tracking/, but removed everything librigidbodytracker related to offer a simple mocap to ros node that publishes a pointcloud and the tracked objects.

This project also uses our own [libmotioncapture](https://gitlab.isse.de/robotik/dynamic-swarms/libmotioncapture) (forked from https://github.com/IMRCLab/libmotioncapture) to be able to publish the whole pointcloud via vicon while also tracking certain objects.

The points of tracked object usually wouldn't get published into the resulting pointcloud.

# Build
```
git clone --recurse-submodules https://gitlab.isse.de/robotik/dynamic-swarms/ros-motioncapture

```
or (if already cloned)

```
git submodule update --init --recursive
```
then
```
colcon build
```
