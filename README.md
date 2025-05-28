# rosbag_manipulation

A set of tools for manipulation ROS2 rosbags in a deterministic manner. For example `ros2 bag convert` doesn't have options for downsampling or cropping the bag within a certain timeframe (even though the latter is implied by the documentation). Thus, I write this code to fill the void.

Currently, this repository is in active development and functionality isn't guaranteed.

## Installation

You will need Python 3 to run this code. It has been used with Python 3.10, but may work with other version. Run the following command to install the repository:
```
pip install .
```

## Example Use

### Downsampling

TBD

### Hertz Analysis
In order to generate histograms with an analysis of the hertz rates for a specific topic, run a command similar to the following:
```
python src/rosbag_manipulation/main.py \
       hertz_analysis /media/dbutterfield3/T73/hercules_test_datasets_V1.1/ausenv_test1_checkpoints_CSLAM_2UAVUGV \
       /home/dbutterfield3/Research/rosbag_manipulation/figures \
       --topic /hercules_node/Drone1/front_center_Scene/image \
       --expected_msgs 4125
```

If you want to investigate the `/tf` or `/tf_static` topic, these contain headers per transform, and thus you need to specify the name of the robot, as show below. Note we assume that transform you want to investigate is `{robot_name}/odom_local`:
```
python src/rosbag_manipulation/main.py \
       hertz_analysis /media/dbutterfield3/T73/hercules_test_datasets_V1.1/ausenv_test1_checkpoints_CSLAM_2UAVUGV \
       /home/dbutterfield3/Research/rosbag_manipulation/figures \
       --topic /tf \
       --robot_name Husky1 \
       --expected_msgs 17059
```

For more information on the command line arguments, use the `--help` argument with the `src/rosbag_manipulation/main.py` file.