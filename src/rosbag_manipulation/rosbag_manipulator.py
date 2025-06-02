import cv2
from collections import defaultdict
from decimal import Decimal
import glob
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.typesys.store import Typestore
import tqdm
import yaml

class rosbag_Manipulator():

    # Initialize using values within the passed dictionary
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)
        self.create_typestore_with_external_msgs()
        self.run_operation()

    @classmethod
    def from_yaml(cls, yaml_path: str):
        with open(yaml_path, "r") as yaml_file:
            yaml_dict = yaml.safe_load(yaml_file)
            return cls(**yaml_dict)
        
    def run_operation(self):
        function = getattr(self, self.operation_to_run)
        function()
        
    # =========================================================================
    # ============================== Typestores =============================== 
    # =========================================================================  

    @staticmethod
    def guess_msgtype(path: Path) -> str:
        """
        Guess message type name from path.
        
        Args:
            path (Path): Path to the message file.

        Returns:
            str: The guessed message type name.
        """
        name = path.relative_to(path.parents[2]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)

    def create_typestore_with_external_msgs(self) -> None:
        """
        Create a Typestore with external message types added from the specified path.
        """
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        
        # Load external message types
        if self.external_msgs_path:
            external_msgs_path = Path(self.external_msgs_path)
            for msg_path in glob.glob(str(external_msgs_path / Path('**') /Path('*.msg')), recursive=True):
                msg_path = Path(msg_path)
                msgdef = msg_path.read_text(encoding='utf-8')
                self.typestore.register(get_types_from_msg(msgdef, rosbag_Manipulator.guess_msgtype(msg_path)))
    
    # =========================================================================
    # ============================ Hertz Analysis ============================= 
    # ========================================================================= 

    @staticmethod
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

    def hertz_analysis(self) -> None:
        """
        Analyze the hertz of various topics in a ROS2 bag file. Will output
        histograms of the hertz between each pair of consecutive messages 
        for the specified topic.
        """

        # Extract operation specific parameters
        topic: str = self.operation_params['hertz_analysis']['topic']
        output_folder: str = self.operation_params['hertz_analysis']['output_folder']
        expected_msgs: int = self.operation_params['hertz_analysis']['expected_msgs']
        try: robot_name: str = self.operation_params['hertz_analysis']['robot_name']
        except: pass

        # Convert string to Path object
        input_path = Path(self.input_bag)

        # Open the bag file for reading
        with Reader(input_path) as reader:
            # Store timestamps
            topic_timestamps = []

            # setup tqdm 
            pbar = tqdm.tqdm(total=expected_msgs, desc="Extracting timestamps", unit="frames")

            # Only analyze the specified topic
            connections = [x for x in reader.connections if x.topic == topic]
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                topic = conn.topic
                msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)

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
        rosbag_Manipulator.create_histogram(
            data=hertz_diffs,
            title=f'Time Differences for Topic: {topic}',
            xlabel='Time Difference (seconds)',
            ylabel='Seq. Message Pairs Count (#)',
            filename=Path(output_folder, f'{robot_str}_{topic.replace("/", "_")}_DIFFS.png')
        )
        rosbag_Manipulator.create_histogram(
            data=hertz_values,
            title=f'Hertz Analysis for Topic: {topic}',
            xlabel='Hertz (Hz)',
            ylabel='Seq. Message Pairs Count (#)',
            filename=Path(output_folder, f'{robot_str}_{topic.replace("/", "_")}_HERTZ.png')
        )

    # =========================================================================
    # ============================= Downsampling ==============================
    # ========================================================================= 

    def downsample(self):
        """
        Downsample a ROS2 bag file by downsampling the frequency of specified topics. Additionally
        includes or excludes unmentioned topics based on `include_unmentioned_topics` parameter.
        """

        # Convert to pathlib paths
        input_bag = Path(self.input_bag)
        output_bag = Path(self.output_bag)

        # Extract operation specific parameters
        topic_downsample_ratios: dict = self.operation_params['downsample']['topics']
        include_unmentioned_topics: bool = self.operation_params['downsample']['include_unmentioned_topics']

        # Ensure we aren't overwriting an existing output bag
        if output_bag.exists():
            raise AssertionError("Delete Output Directory first!")
        
        # Calculate downsample ratios
        for topic in topic_downsample_ratios:
            if topic_downsample_ratios[topic] <= 0:
                raise ValueError("Downsample rates must be greater than 0.")
            topic_downsample_ratios[topic] = 1 / topic_downsample_ratios[topic]

        # Open the bag
        with Reader(input_bag) as reader:
            connections = reader.connections
            
            # Create a writer with only the specified topics
            with Writer(output_bag, version=5) as writer:
                conn_map = {}
                for conn in connections:
                    if conn.topic in topic_downsample_ratios:
                        conn_map[conn.topic] = writer.add_connection(
                            topic=conn.topic,
                            msgtype=conn.msgtype,
                            msgdef=conn.msgdef,
                            typestore=self.typestore,
                            serialization_format='cdr',
                            offered_qos_profiles=conn.ext.offered_qos_profiles
                        )

                # Initialize counters for each topic
                topic_counters = defaultdict(int)
                for topic in topic_downsample_ratios:
                    topic_counters[topic] = 1

                # Setup progress bars
                pbarC = tqdm.tqdm(total=None, desc="Downsampling Requested Messages", unit=" messages")
                pbarW = tqdm.tqdm(total=None, desc="Writing Messages", unit=" messages")

                # Interate through messages in the bag
                connections = [x for x in reader.connections if x.topic in conn_map]
                for conn, timestamp, rawdata in reader.messages(connections=connections):

                    # Check its a topic we want to write
                    if (conn.topic in topic_downsample_ratios) or include_unmentioned_topics:
                        pbarC.update(1)

                        # Extract the downsample ratio for this topic
                        try:
                            downsample_ratio = topic_downsample_ratios[conn.topic]
                        except KeyError:
                            # Assume no downsampling for unmentioned topics
                            downsample_ratio = 1.0

                        # See if we need to save this message or cut it
                        if topic_counters[conn.topic] >= topic_downsample_ratios[conn.topic]:
                            pbarW.update(1)

                            # Subtract integer portion of count
                            topic_counters[conn.topic] -= topic_downsample_ratios[conn.topic]
                            
                            # Write the raw data
                            writer.write(conn_map[conn.topic], timestamp, rawdata)

                        # Increment the counter for this topic
                        topic_counters[conn.topic] += 1

                # Close the progress bars
                pbarC.close()
                pbarW.close()
        
        print(f"Downsampling complete. Output bag saved to {output_bag}.")

    # =========================================================================
    # ============================== Cropping ================================
    # ========================================================================= 

    def crop(self):
        """
        Crop a ROS2 bag file to only include messages within the specified time range. Note
        that this function uses the timestamps of when the messages were written to the bag,
        not the timestamps in the headers of the messages themselves.
        """

        # Extract operation specific parameters
        start_ts: float = self.operation_params['crop']['start_ts']
        end_ts: float = self.operation_params['crop']['end_ts']

        # Convert to pathlib paths
        input_bag = Path(self.input_bag)
        output_bag = Path(self.output_bag)

        # Make sure we aren't overwriting an existing output bag
        if output_bag.exists():
            raise AssertionError("Delete Output Directory first!")

        with Reader(input_bag) as reader:
            connections = reader.connections
            
            # Setup the bag writer
            with Writer(output_bag, version=5) as writer:
                conn_map = {}
                for conn in connections:
                    conn_map[conn.topic] = writer.add_connection(
                        topic=conn.topic,
                        msgtype=conn.msgtype,
                        msgdef=conn.msgdef,
                        typestore=self.typestore,
                        serialization_format='cdr',
                        offered_qos_profiles=conn.ext.offered_qos_profiles
                    )

                # setup tqdm 
                pbarW = tqdm.tqdm(total=None, desc="Writing messages", unit=" messages")

                # Iternate through messages in the bag
                for conn, timestamp, rawdata in reader.messages(start=start_ts * 1e9, stop=end_ts * 1e9):
                    writer.write(conn_map[conn.topic], timestamp, rawdata)
                    pbarW.update(1)