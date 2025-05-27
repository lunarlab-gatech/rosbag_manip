# rosbag_manipulation

A set of tools for manipulation rosbags in a deterministic manner. For example `ros2 bag convert` doesn't have options for downsampling or cropping the bag within a certain timeframe (even though the latter is implied by the documentation). Thus, I write this code to fill the void.

Currently, this repository is in active development and functionality isn't guaranteed.