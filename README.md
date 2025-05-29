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

See use cases for this repository below. For more information on the command line arguments, use the `--help` argument with the `src/rosbag_manipulation/main.py` file.

### Downsampling

This functionality allows downsampling the hertz rates of topics in the bag (for example, 35 Hz to 5 Hz). It additionally resizes images to half their width/height, resulting in an image with 1/4 of the data. See an example command below for this functionality:
```
python src/rosbag_manipulation/main.py \
       downsample /media/dbutterfield3/T73/hercules_test_datasets_V1.1/ausenv_test1_checkpoints_CSLAM_2UAVUGV \
       /media/dbutterfield3/T73/hercules_test_datasets_V1.1/ausenv_test1_checkpoints_CSLAM_2UAVUGV_downsampled \
       --external_msgs_path /home/dbutterfield3/Research/rosbag_manipulation/external_msgs \
       --topics /hercules_node/Husky1/odom_local \
                /hercules_node/Husky2/odom_local \
                /hercules_node/Drone1/odom_local \
                /hercules_node/Drone2/odom_local \
                /hercules_node/Husky1/front_center_Scene/image \
                /hercules_node/Husky1/front_center_Scene/camera_info \
                /hercules_node/Husky1/front_center_DepthPerspective/image \
                /hercules_node/Husky1/front_center_DepthPerspective/camera_info \
                /hercules_node/Husky2/front_center_Scene/image \
                /hercules_node/Husky2/front_center_Scene/camera_info \
                /hercules_node/Husky2/front_center_DepthPerspective/image \
                /hercules_node/Husky2/front_center_DepthPerspective/camera_info \
                /hercules_node/Drone1/front_center_Scene/image \
                /hercules_node/Drone1/front_center_Scene/camera_info \
                /hercules_node/Drone1/front_center_DepthPerspective/image \
                /hercules_node/Drone1/front_center_DepthPerspective/camera_info \
                /hercules_node/Drone2/front_center_Scene/image \
                /hercules_node/Drone2/front_center_Scene/camera_info \
                /hercules_node/Drone2/front_center_DepthPerspective/image \
                /hercules_node/Drone2/front_center_DepthPerspective/camera_info \
       --downsample_rates   1.0 1.0   1.0 1.0 \
                          0.125 1.0 0.125 1.0 \
                          0.125 1.0 0.125 1.0 \
                          0.125 1.0 0.125 1.0 \
                          0.125 1.0 0.125 1.0
```

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

### Crop
A rosbag can be cropped to only include message written into the bag within a certain timeframe with this feature. Note that this doesn't take into account the timestamps inside the messages. An example command for this functionality is as follows:
```
python src/rosbag_manipulation/main.py \
       crop /media/dbutterfield3/T73/hercules_test_datasets_V1.1/ausenv_test1_checkpoints_CSLAM_2UAVUGV \
       /media/dbutterfield3/T73/hercules_test_datasets_V1.1/ausenv_test1_checkpoints_CSLAM_2UAVUGV_only60 \
       --external_msgs_path /home/dbutterfield3/Research/rosbag_manipulation/external_msgs \
       --start_ts 1747048365.868941009 \
       --end_ts 1747048425.868941009 \
```