# rosbag_manipulation

[![Python Unit Tests](https://github.com/lunarlab-gatech/rosbag_manip/actions/workflows/python_test.yml/badge.svg?branch=master)](https://github.com/lunarlab-gatech/rosbag_manip/actions/workflows/python_test.yml) [![Coverage Status](https://coveralls.io/repos/github/lunarlab-gatech/rosbag_manip/badge.svg?branch=master)](https://coveralls.io/github/lunarlab-gatech/rosbag_manip?branch=master)

A set of tools for manipulation ROS2 rosbags in a deterministic manner, meant to fill a void in current tools. For example, `ros2 bag convert` doesn't have options for downsampling or cropping the bag within a certain timeframe (even though the latter is implied by the documentation). 

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


Note that support for more messages can be easily added by altering `get_mapping()` and `msg_mapping_ros2_to_ros1` in `rosbag_manip.py`.
