from collections import defaultdict
from decimal import Decimal
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore
import tqdm

def create_histogram(data: list, title: str, xlabel: str, ylabel: str, filename: str) -> None:
    """
    Create and save a histogram from the provided data.

    Args:
        data (list): Data to create the histogram from.
        title (str): Title of the histogram.
        xlabel (str): Label for the x-axis.
        ylabel (str): Label for the y-axis.
        filename (str): Filename to save the histogram image.
    """
    plt.figure(figsize=(10, 6))
    plt.hist(data, bins=100, color='blue', alpha=0.7)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.grid(True)
    plt.yscale('log')

    # Save the figure
    if not Path(filename).parent.exists():
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(filename)

def analyze_hertz(input_path: str, output_folder: str, topic: str, expected_msgs: int = None, robot_name: str = None) -> None:
    """
    Analyze the hertz of various topics in a ROS2 bag file. Will output
    histograms of the hertz between each pair of consecutive messages 
    for the specified topic.

    Args:
        input_path (str): Path to the input ROS2 bag folder.
        output_folder (str): Location to save generated figures.
        topic (str): Topic to analyze.
        expected_msgs (int): The expected number of messages to find,
            for use with tqdm.
        robot_name (str): If topic is /tf or /tf_static, used to get 
            the hertz values for the transform for the specific robot.
            Assumes the transform is on '{robot_name}/odom_local'.
    """

    # Convert string to Path object
    input_path = Path(input_path)

    # Open the bag file for reading
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with Reader(input_path) as reader:
        # Store timestamps
        topic_timestamps = []

        # setup tqdm 
        pbar = tqdm.tqdm(total=expected_msgs, desc="Extracting timestamps", unit="frames")

        # Only analyze the specified topic
        connections = [x for x in reader.connections if x.topic == topic]
        for conn, timestamp, rawdata in reader.messages(connections=connections):
            topic = conn.topic
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

            if hasattr(msg, 'transforms'):
                for transform in msg.transforms:
                    if transform.child_frame_id == robot_name+"/odom_local" \
                       and hasattr(transform, 'header') and hasattr(transform.header, 'stamp'):
                        topic_timestamps.append(Decimal(transform.header.stamp.sec) + Decimal(transform.header.stamp.nanosec) * Decimal(1e-9))
            elif hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                topic_timestamps.append(Decimal(msg.header.stamp.sec) + Decimal(msg.header.stamp.nanosec) * Decimal(1e-9))
            else:
                raise ValueError(f"Topic: {topic} does not have a valid header with timestamp.")
            pbar.update(1)

    # Calculate the differences between consecutive timestamps
    if len(topic_timestamps) < 2:
        raise ValueError(f"Not enough messages in topic {topic} to analyze hertz.")
    hertz_diffs = [topic_timestamps[i] - topic_timestamps[i - 1] for i in range(1, len(topic_timestamps))]
    hertz_values = [1 / diff for diff in hertz_diffs if diff > 0]

    # Sort each of these lists
    hertz_diffs.sort()
    hertz_values.sort()

    # To remove potentially noisy values, remove first and last 5
    if len(hertz_diffs) > 5:
        hertz_diffs = hertz_diffs[5:-5]
        hertz_values = hertz_values[5:-5]

    # Create histograms for the hertz values and time differences
    robot_str = ""
    if topic == "/tf" or topic == "/tf_static":
        robot_str = robot_name
    create_histogram(
        data=hertz_diffs,
        title=f'Time Differences for Topic: {topic}',
        xlabel='Time Difference (seconds)',
        ylabel='Seq. Message Pairs Count (#)',
        filename=Path(output_folder, f'{robot_str}_{topic.replace("/", "_")}_DIFFS.png')
    )
    create_histogram(
        data=hertz_values,
        title=f'Hertz Analysis for Topic: {topic}',
        xlabel='Hertz (Hz)',
        ylabel='Seq. Message Pairs Count (#)',
        filename=Path(output_folder, f'{robot_str}_{topic.replace("/", "_")}_HERTZ.png')
    )