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
then build (Vicon and OptiTrack enabled by default):
```bash
colcon build
# Optional: enable VRPN, including its test
colcon build --cmake-args -DLIBMOTIONCAPTURE_ENABLE_VRPN=ON
```

# Test
After building:
```bash
source install/setup.bash
colcon test --packages-select ros_motioncapture --event-handlers console_direct+
colcon test-result --verbose
```
The mock test checks point-cloud messages, a configured rigid body's TF,
diagnostics, and clean shutdown after SIGINT. No mocap hardware is needed.
When VRPN is enabled, its test starts a local tracker server on port 43883 and checks that its
position and rotation arrive correctly in TF, followed by clean shutdown.

GitHub Actions runs the same build and tests in a ROS Lyrical container on pushes
to master and pull requests targeting master. It can also be started manually
from the Actions tab.
CI explicitly enables VRPN to run both tests.
