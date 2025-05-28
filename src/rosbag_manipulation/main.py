import argparse
from rosbag_manipulation.downsample import downsample_bag
from rosbag_manipulation.hertz_analysis import analyze_hertz

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('functionality', choices=['downsample', 'hertz_analysis'],
                        help='Choose the functionality to execute: downsample or hertz_analysis')
    parser.add_argument('input', help='Path to input ROS2 bag folder')
    parser.add_argument('output', help='Path to output ROS2 bag folder (if functionality is downsample), or path to the directory for saving figures (if functionality is hertz_analysis).')
    parser.add_argument('--topic', help='Topic to analyze (if functionality is hertz_analysis).')
    parser.add_argument('--expected_msgs', type=int, help='Expected number of messages for the --topic (if functionality is hertz_analysis). This is optional and will setup a tqdm progress bar for the analysis.')
    parser.add_argument('--robot_name', type=str, help="If --topic is '/tf' or '/tf_static', use this as the robot name for the following transform '{robot_name}/odom_local'")
    args = parser.parse_args()

    # Check for conditionally required arguments
    if args.functionality == 'downsample' and args.output is None:
        raise ValueError("Output path is required for downsampling functionality.")
    if args.functionality == 'hertz_analysis' and args.topic is None:
        raise ValueError("One topic must be specified for hertz analysis functionality.")
    if args.functionality == 'hertz_analysis' and (args.topic == '/tf' or args.topic == '/tf_static') \
        and args.robot_name is None:
        raise ValueError("Robot name must be specified if topic is /tf or /tf_static")
    
    # Call the appropriate function based on the chosen functionality
    if args.functionality == 'downsample':
        downsample_bag(args.input, args.output)
    elif args.functionality == 'hertz_analysis':
        analyze_hertz(args.input, args.output, args.topic, args.expected_msgs, args.robot_name)
    else:
        raise ValueError("Invalid functionality selected.")

if __name__ == '__main__':
    main()