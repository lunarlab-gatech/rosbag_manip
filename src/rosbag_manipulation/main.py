import argparse
from rosbag_manipulation.downsample import downsample_bag
from rosbag_manipulation.hertz_analysis import analyze_hertz
from rosbag_manipulation.crop import crop_bag
from rosbag_manipulation.typestore import get_typestore_with_external_msgs

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('functionality', choices=['downsample', 'hertz_analysis', 'crop'],
                        help='Choose the functionality to execute: downsample, hertz_analysis, or crop')
    parser.add_argument('input', help='Path to input ROS2 bag folder')
    parser.add_argument('output', help='Path to output ROS2 bag folder (if functionality is downsample or crop), or path to the directory for saving figures (if functionality is hertz_analysis).')
    parser.add_argument('--external_msgs_path', type=str, default=None, help='Path to folder with external message types to be added to the typestore.')
    parser.add_argument('--topic', help='Topic to analyze (if functionality is hertz_analysis).')
    parser.add_argument('--expected_msgs', type=int, help='Expected number of messages for the --topic (if functionality is hertz_analysis). This is optional and will be used for tqdm progress bars.')
    parser.add_argument('--robot_name', type=str, help="If --topic is '/tf' or '/tf_static', use this as the robot name for the following transform '{robot_name}/odom_local' (if functionality is hertz_analysis).")
    parser.add_argument('--start_ts', type=float, default=0.0, help='Start timestamp in seconds for cropping the bag (if functionality is crop). Default is 0.0 seconds.')
    parser.add_argument('--end_ts', type=float, default=None, help='End timestamp in seconds for cropping the bag (if functionality is crop). Default is None, which means no end timestamp is set.')
    args = parser.parse_args()

    # Check for conditionally required arguments
    if args.functionality == 'downsample' and args.output is None:
        raise ValueError("Output path is required for downsampling functionality.")
    if args.functionality == 'hertz_analysis' and args.topic is None:
        raise ValueError("One topic must be specified for hertz analysis functionality.")
    if args.functionality == 'hertz_analysis' and (args.topic == '/tf' or args.topic == '/tf_static') \
        and args.robot_name is None:
        raise ValueError("Robot name must be specified if topic is /tf or /tf_static")
    
    # Load the typestore with external message types if provided
    typestore = get_typestore_with_external_msgs(args.external_msgs_path)

    # Call the appropriate function based on the chosen functionality
    if args.functionality == 'downsample':
        downsample_bag(args.input, args.output)
    elif args.functionality == 'hertz_analysis':
        analyze_hertz(args.input, args.output, typestore, args.topic, args.expected_msgs, args.robot_name)
    elif args.functionality == 'crop':
        crop_bag(args.input, args.output, typestore, args.start_ts, args.end_ts)
    else:
        raise ValueError("Invalid functionality selected.")

if __name__ == '__main__':
    main()