# rosbag_manipulation

[![Python Unit Tests](https://github.com/lunarlab-gatech/rosbag_manip/actions/workflows/python_test.yml/badge.svg?branch=master)](https://github.com/lunarlab-gatech/rosbag_manip/actions/workflows/python_test.yml) [![Coverage Status](https://coveralls.io/repos/github/lunarlab-gatech/rosbag_manip/badge.svg?branch=master)](https://coveralls.io/github/lunarlab-gatech/rosbag_manip?branch=master)

A set of tools for manipulation ROS2 bags in a deterministic manner, meant to fill a void in current tools. For example, `ros2 bag convert` doesn't have options for downsampling or cropping the bag within a certain timeframe (even though the latter is implied by the documentation). 

Currently, the library assumes the ROS2 bags have messages from ROS2 Humble, and for converting to ROS1 bags, the output bag will be compatible with ROS1 Noetic.

**WARNING:** Currently, this repository is in active development and functionality isn't guaranteed to work. If you will depend on this repository for important tasks, perhaps write test cases for the corresponding functionality before deployment/use.

## Installation

You will need Python 3.10+ to run this code. Run the following commands to install the repository:
```
git submodule init
git submodule update
pip install .
```

## Example Use

See use cases for this repository below. For more information on all the specific operation parameters, see the `config/explanation.yaml` file.

### Downsampling

This functionality allows downsampling the hertz rates of topics in the bag (for example, 40 Hz to 5 Hz). It can also prune topics by setting `include_unmentioned_topics` to false. See an example command below for this functionality:
```
rosbag_manip <repository_path>/config/downsample.yaml
```

### Hertz Analysis
In order to generate histograms with an analysis of the hertz rates for a specific topic, run a command similar to the following:
```
rosbag_manip <repository_path>/config/hertz_analysis.yaml
```

If you want to investigate the `/tf` or `/tf_static` topic, these contain headers per transform, and thus you need to specify the name of the robot, as show below. Note we assume that transform you want to investigate is `{robot_name}/odom_local`:
```
rosbag_manip <repository_path>/config/hertz_analysis_tf.yaml
```

### Crop
A rosbag can be cropped to only include message written into the bag within a certain timeframe with this feature. Note that this doesn't take into account the timestamps inside the messages. An example command for this functionality is as follows:
```
rosbag_manip <repository_path>/config/crop.yaml
```

### ROS2 to ROS1 conversion
A ROS2 bag can be converted to ROS1 with this functionality, note that only certain message types are supported. When running the command, any unsuported message types will print a warning message. 
```
rosbag_manip <repository_path>/config/convert_ros2_to_ros1.yaml
```

Note that support for more messages can be easily added by altering `get_mapping()` and `msg_mapping_ros2_to_ros1` in `rosbag_manip.py`. Here is a list of all currently supported msg types:
```python
"builtin_interfaces/msg/Time"
"geometry_msgs/msg/Point"
"geometry_msgs/msg/Pose"
"geometry_msgs/msg/PoseWithCovariance"
"geometry_msgs/msg/Quaternion"
"geometry_msgs/msg/Transform"
"geometry_msgs/msg/TransformStamped"
"geometry_msgs/msg/Twist"
"geometry_msgs/msg/TwistWithCovariance"
"geometry_msgs/msg/Vector3"
"nav_msgs/msg/Odometry"
"sensor_msgs/msg/Image"
"sensor_msgs/msg/Imu"
"sensor_msgs/msg/CameraInfo"
"sensor_msgs/msg/RegionOfInterest"
"std_msgs/msg/Header"
"tf2_msgs/msg/TFMessage"
```

## Validation

### Unit Tests & Coverage

Run the following command to run the unit tests and generate a code coverage report:
```
coverage run --source rosbag_manip -m unittest discover tests/ -v
coverage report
coverage html
```

### Profiling

Run the following command to profile the code (via the unit tests):
```
python -m cProfile -o profile.out -m unittest discover tests/ -v
snakeviz profile.out
```
