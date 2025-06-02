# rosbag_manipulation

A set of tools for manipulation ROS2 rosbags in a deterministic manner. For example `ros2 bag convert` doesn't have options for downsampling or cropping the bag within a certain timeframe (even though the latter is implied by the documentation). Thus, I write this code to fill the void.

**WARNING:** Currently, this repository is in active development and functionality isn't guaranteed to work. If you will depend on this repository for important tasks, perhaps write test cases for the corresponding functionality before deployment/use.

## Installation

You will need Python 3 to run this code. It has been used with Python 3.10, but may work with other version. Run the following commands to install the repository:
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