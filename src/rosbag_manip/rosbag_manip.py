from collections import defaultdict
from .data_types.ImageData import ImageData
from .data_types.OdometryData import OdometryData
from decimal import Decimal
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from .rosbag.Ros2BagWrapper import Ros2BagWrapper
from rosbags.rosbag2 import Reader as Reader2
from rosbags.rosbag2 import Writer as Writer2
from scipy.spatial.transform import Rotation as R
import tqdm
import yaml

class rosbag_manipulation():

    def __init__(self, **kwargs):
        """
        Initialize the rosbag manipulator with the provided keyword arguments,
        which is assumed to be a dictionary containing the manipulation configuration.

        Args:
            **kwargs: Keyword arguments containing the manipulation configuration, including:
                - 'input_bag': Path to the input ROS2 bag file.
                - 'output_bag': Path to the output ROS2 bag file, if necessary.
                - 'operation_params': Dictionary containing operation parameters for specific manipulations.
                - 'external_msgs_path_ros2': Path to the directory containing external message definitions.
                - 'operation_to_run': The name of the operation to run, which should be a method of this class.
        """

        # Assign attributes from input arguments
        for key, value in kwargs.items():
            setattr(self, key, value)

        # Create a ROS2 bag wrapper
        self.bag_wrapper = Ros2BagWrapper(self.input_bag, self.external_msgs_path_ros2)

        # Setup dictionary to store msg instances
        self.ros1_msg_instances = {}

        # Run desired operation
        self.run_operation()

    @classmethod
    def from_yaml(cls, yaml_path: str):
        """
        Create a rosbag manipulator from a YAML file.

        Args:
            yaml_path (str): Path to the YAML file containing the manipulation configuration.
        """
        with open(yaml_path, "r") as yaml_file:
            yaml_dict = yaml.safe_load(yaml_file)
            return cls(**yaml_dict)
        
    def run_operation(self):
        """
        Load the operation to run from 'self.operation_to_run' and execute it.

        Used Attributes:
            self.operation_to_run (str): The name of the operation to run, which should be a method of this class.
        """
        function = getattr(self, self.operation_to_run)
        function()

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
        a histogram of the hertz for each pair of consecutive messages 
        for the specified topic, and another histogram of the time differences.

        Used Attributes:
            self.input_bag (str): Path to the input ROS2 bag file.
            self.operation_params (dict): Dictionary containing operation parameters, including:
                - 'hertz_analysis': Dictionary with keys:
                    - 'topic': The topic to analyze.
                    - 'output_folder': Folder to save the output histograms.
                    - 'expected_msgs': Expected number of messages in the topic, for progress bar.
                    - 'robot_name': if 'topic' is '/tf' or '/tf_static', the robot name which then
                        choses the transform with a child frame id of '{robot_name}/odom_local'.
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
        with Reader2(input_path) as reader:
            # Store timestamps
            topic_timestamps = []

            # setup tqdm 
            pbar = tqdm.tqdm(total=expected_msgs, desc="Extracting timestamps", unit="frames")

            # Only analyze the specified topic
            connections = [x for x in reader.connections if x.topic == topic]
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                topic = conn.topic
                msg = self.bag_wrapper.get_typestore().deserialize_cdr(rawdata, conn.msgtype)

                if hasattr(msg, 'transforms'):
                    for transform in msg.transforms:
                        if transform.child_frame_id == robot_name+"/odom_local" \
                        and hasattr(transform, 'header') and hasattr(transform.header, 'stamp'):
                            topic_timestamps.append(Decimal(transform.header.stamp.sec) + Decimal(transform.header.stamp.nanosec) * Decimal(1e-9))
                elif hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    topic_timestamps.append(rosbag_manipulation.extract_timestamp(msg))
                else:
                    raise ValueError(f"Topic: {topic} does not have a valid header with timestamp.")
                pbar.update(1)

        # Calculate the differences between consecutive timestamps
        if len(topic_timestamps) < 2:
            raise ValueError(f"Not enough messages in topic {topic} to analyze hertz.")
        hertz_diffs = [topic_timestamps[i] - topic_timestamps[i - 1] for i in range(1, len(topic_timestamps))]
        hertz_values = [1 / diff for diff in hertz_diffs if diff > 0]

        # Output a message if some differences are zero
        num_seq_messages_with_same_timestamps = len([x for x in hertz_diffs if x == 0])
        if num_seq_messages_with_same_timestamps > 0:
            print(f"Warning: Sequential Pairs of timestamps are equivalent {num_seq_messages_with_same_timestamps} times. Excluding from plot...")

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
        rosbag_manipulation.create_histogram(
            data=hertz_diffs,
            title=f'Time Differences for Topic: {topic}',
            xlabel='Time Difference (seconds)',
            ylabel='Seq. Message Pairs Count (#)',
            filename=Path(output_folder, f'{robot_str}_{topic.replace("/", "_")}_DIFFS.png')
        )
        rosbag_manipulation.create_histogram(
            data=hertz_values,
            title=f'Hertz Analysis for Topic: {topic}',
            xlabel='Hertz (Hz)',
            ylabel='Seq. Message Pairs Count (#)',
            filename=Path(output_folder, f'{robot_str}_{topic.replace("/", "_")}_HERTZ.png')
        )

    # =========================================================================
    # ============================ View IMU Data ==============================
    # ========================================================================= 

    def view_imu_data(self):
        """
        Plot the linear acceleration data of an IMU topic in a ROS2 bag.

        Used Attributes:
            self.input_bag (str): Path to the input ROS2 bag file.
            self.operation_params (dict): Dictionary containing operation parameters, including:
                - 'view_imu_data': Dictionary with keys:
                    - 'topic': The topic to read the IMU data from; must be 'sensor_msgs/msg/Imu'.
                    - 'output_folder': The folder to save plotted IMU data.
                    - 'expected_msgs': Expected number of messages in the topic, for progress bar.
                            Optional.
        """

        # Extract operation specific parameters
        topic: str = self.operation_params['view_imu_data']['topic']
        output_folder: str = self.operation_params['view_imu_data']['output_folder']
        try: expected_msgs: int = self.operation_params['view_imu_data']['expected_msgs']
        except: expected_msgs = None
        try: 
            data_range = self.operation_params['view_imu_data']['data_range']
            assert len(data_range) == 2
            data_range = tuple(data_range)
        except: data_range = None

        # Create the output folder if it doesn't exist
        Path(output_folder).mkdir(parents=True, exist_ok=True)

        # Check if we've already parsed the bag into a .npy file
        npy_path = Path(output_folder) / "imu_data.npy"
        if npy_path.exists():
            
            # If so, load the cached data
            data = np.load(npy_path, allow_pickle=True).item()
            timestamps = data['timestamps']
            lin_accel = data['lin_accel']
            ang_vel = data['ang_vel']
            orientation = data['orientation']

        else: # Otherwise, read the bag

            # Convert string to Path object
            input_path = Path(self.input_bag)
            
            # Initialize lists
            lin_accel = {'x': [], 'y': [], 'z': []}
            ang_vel = {'x': [], 'y': [], 'z': []}
            orientation = {'x': [], 'y': [], 'z': [], 'w': []}
            timestamps = []

            # Open the bag file for reading
            with Reader2(input_path) as reader:

                # setup tqdm 
                pbar = tqdm.tqdm(total=expected_msgs, desc="Extracting IMU Data", unit=" frames")

                # Only analyze the specified topic
                connections = [x for x in reader.connections if x.topic == topic]
                if len(connections) != 1:
                    raise ValueError("Input topic doesn't exist in bag!")

                # Iterate through messages
                for conn, timestamp, rawdata in reader.messages(connections=connections):
                    msg = self.bag_wrapper.get_typestore().deserialize_cdr(rawdata, conn.msgtype)
                    
                    # Timestamp
                    timestamps.append(Decimal(msg.header.stamp.sec) + Decimal(msg.header.stamp.nanosec) * Decimal(1e-9))

                    # Linear acceleration
                    lin_accel['x'].append(msg.linear_acceleration.x)
                    lin_accel['y'].append(msg.linear_acceleration.y)
                    lin_accel['z'].append(msg.linear_acceleration.z)

                    # Angular velocity
                    ang_vel['x'].append(msg.angular_velocity.x)
                    ang_vel['y'].append(msg.angular_velocity.y)
                    ang_vel['z'].append(msg.angular_velocity.z)

                    # Orientation
                    orientation['x'].append(msg.orientation.x)
                    orientation['y'].append(msg.orientation.y)
                    orientation['z'].append(msg.orientation.z)
                    orientation['w'].append(msg.orientation.w)

                    pbar.update(1)

            # Convert quaternions into euler angles
            quaternions = np.array([ orientation['x'], orientation['y'],
                orientation['z'], orientation['w']]).T 
            euler_angles = R.from_quat(quaternions).as_euler('xyz', degrees=True)
            orientation = {'roll': euler_angles[:, 0], 'pitch': euler_angles[:, 1], 'yaw': euler_angles[:, 2]}

            # Save extracted data into a .npy file
            np.save(Path(output_folder) / "imu_data.npy", {
                'timestamps': timestamps,
                'lin_accel': lin_accel,
                'ang_vel': ang_vel,
                'orientation': orientation,
            })
            
        def multi_list_plotter(data_dict: dict, timestamps: list, data_range: tuple[int], 
                               title: str, ylabel: str, filename: str):
            """
            Helper function that plots each list in a dictionary in its own subplot,
            all in a single matplotlib figure.
            """

            num_plots = len(data_dict)
            fig, axes = plt.subplots(num_plots, 1, figsize=(10, 3 * num_plots), sharex=True)

            # Make it a list always
            if num_plots == 1:
                axes = [axes]

            shift = timestamps[0]
            for i in range(0, len(timestamps)):
                timestamps[i] -= shift

            if data_range is not None:
                timestamps = timestamps[data_range[0]:data_range[1]]

            for ax, (key, values) in zip(axes, data_dict.items()):
                if data_range is not None:
                    values = values[data_range[0]:data_range[1]]
                ax.plot(timestamps, values, label=key)
                ax.set_ylabel(f"{key} - ({ylabel})")
                ax.grid(True)
                ax.legend(loc='upper right')

            axes[-1].set_xlabel("Time (s)")
            fig.suptitle(f"{title} vs Time", fontsize=14)
            fig.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.savefig(str(Path(output_folder) / filename))
            plt.close()

        # Create all plots
        multi_list_plotter(lin_accel, timestamps, data_range, "Linear Acceleration", "m/s^2", "linear_acceleration.png")
        multi_list_plotter(ang_vel, timestamps, data_range, "Angular Velocity", "rad/s", "angular_velocity.png")
        multi_list_plotter(orientation, timestamps, data_range, "Orientation", "degrees", "orientation.png")


    # =========================================================================
    # ============================= Downsampling ==============================
    # ========================================================================= 

    def downsample(self):
        """
        Downsample a ROS2 bag file by downsampling the frequency of specified topics. Additionally
        includes or excludes unmentioned topics based on `include_unmentioned_topics` parameter, which
        can be used to prune topics.
         
        If the number of msgs on a topic multiplied by the downsample ratio is not an integer
        (ex. 696 * 0.1 = 69.6), then the number of messages will be the nearest integer of the 
        calculated float (ex. np.round(69.6) == 70).

        Used Attributes:
            self.input_bag (str): Path to the input ROS2 bag file.
            self.output_bag (str): Path to the output ROS2 bag file.
            self.operation_params (dict): Dictionary containing operation parameters, including:
                - 'downsample': Dictionary with keys:
                    - 'topics': A dictionary mapping topic names to their downsample ratios.
                    - 'include_unmentioned_topics': Boolean indicating whether to include topics not 
                        mentioned in the downsample list in the output bag.
        """

        # Convert to pathlib paths
        input_bag = Path(self.input_bag)
        output_bag = Path(self.output_bag)

        # Extract operation specific parameters
        topic_downsample_ratios: dict = self.operation_params['downsample']['topics'].copy()
        include_unmentioned_topics: bool = self.operation_params['downsample']['include_unmentioned_topics']
        if include_unmentioned_topics:
            raise NotImplementedError('Logic for this isnt fully tested')

        # Ensure we aren't overwriting an existing output bag
        if output_bag.exists():
            raise AssertionError("Delete Output Directory first!")
        
        # Check for valid input downsample ratios
        for topic in topic_downsample_ratios:
            ratio = topic_downsample_ratios[topic]
            if ratio <= 0 or ratio > 1:
                raise ValueError("Downsample rates must be >0 and <= 1")

        # Open the metadata.yaml file and get number of messages on each of the topics
        metadata_path = Path(input_bag / 'metadata.yaml')
        if not metadata_path.exists():
            raise FileNotFoundError(f"Unable to find metadata.yaml in ROS2 bag: {metadata_path}")
        with open(metadata_path, 'r') as f:
            metadata = yaml.safe_load(f)

        topic_counts = {}
        for topic_info in metadata.get('rosbag2_bagfile_information').get('topics_with_message_count'):
            topic_name = topic_info['topic_metadata']['name']
            message_count = topic_info['message_count']
            topic_counts[topic_name] = message_count

        # Calculate which messages we want to save for each topic
        indicies_to_save = {}
        for topic in topic_counts:
            if topic in topic_downsample_ratios:
                desired_final_count = int(np.round(topic_counts[topic] * topic_downsample_ratios[topic]))
                indicies_to_save[topic] = set(np.linspace(0, topic_counts[topic] - 1, num=desired_final_count, dtype=int).tolist())
                
        # Open the bag
        with Reader2(input_bag) as reader:
            connections = reader.connections
            
            # Create a writer with only the specified topics
            with Writer2(output_bag, version=5) as writer:
                conn_map = {}
                for conn in connections:
                    if (conn.topic in topic_downsample_ratios) or include_unmentioned_topics:
                        conn_map[conn.topic] = writer.add_connection(
                            topic=conn.topic,
                            msgtype=conn.msgtype,
                            msgdef=conn.msgdef,
                            typestore=self.bag_wrapper.get_typestore(),
                            serialization_format='cdr',
                            offered_qos_profiles=conn.ext.offered_qos_profiles
                        )

                # Initialize counters for each topic
                topic_counters = defaultdict(int)
                for topic in topic_downsample_ratios:
                    topic_counters[topic] = 0

                # Setup progress bars
                pbarC = tqdm.tqdm(total=None, desc="Reading Requested Messages", unit=" messages")
                pbarW = tqdm.tqdm(total=None, desc="Writing Messages", unit=" messages")

                # Iterate through reader only through desired topics
                if include_unmentioned_topics:
                    connections = reader.connections
                else:
                    connections = [x for x in reader.connections if x.topic in conn_map]

                # Interate through messages in the bag
                for conn, timestamp, rawdata in reader.messages(connections=connections):
                    pbarC.update(1)

                    # See if we need to save this message or cut it
                    if topic_counters[conn.topic] in indicies_to_save[conn.topic]:
                        pbarW.update(1)
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

        Used Attributes:
            self.input_bag (str): Path to the input ROS2 bag file.
            self.output_bag (str): Path to the output ROS2 bag file.
            self.operation_params (dict): Dictionary containing operation parameters, including:
                - 'crop': Dictionary with keys:
                    - 'start_ts': Start timestamp in seconds for cropping.
                    - 'end_ts': End timestamp in seconds for cropping.
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

        with Reader2(input_bag) as reader:
            connections = reader.connections
            
            # Setup the bag writer
            with Writer2(output_bag, version=5) as writer:
                conn_map = {}
                for conn in connections:
                    conn_map[conn.topic] = writer.add_connection(
                        topic=conn.topic,
                        msgtype=conn.msgtype,
                        msgdef=conn.msgdef,
                        typestore=self.bag_wrapper.get_typestore(),
                        serialization_format='cdr',
                        offered_qos_profiles=conn.ext.offered_qos_profiles
                    )

                # setup tqdm 
                pbarW = tqdm.tqdm(total=None, desc="Writing messages", unit=" messages")

                # Iternate through messages in the bag
                for conn, timestamp, rawdata in reader.messages(start=start_ts * 1e9, stop=end_ts * 1e9):
                    writer.write(conn_map[conn.topic], timestamp, rawdata)
                    pbarW.update(1)

    # =========================================================================
    # =========================== Other Utilities =============================
    # ========================================================================= 

    def extract_odometry_to_csv(self):
        """ Extract odometry from a ROS2 bag and save in a csv file. """

        topic: str = self.operation_params['extract_odometry_to_csv']['topic']
        output_folder: str = self.operation_params['extract_odometry_to_csv']['output_file']
        odom_data = OdometryData.from_ros2_bag(self.input_bag, topic)
        odom_data.to_csv(output_folder)

    def extract_images_to_npy(self):
        """ Extract images from a ROS2 bag and saves them into .npy files. """

        topic: str = self.operation_params['extract_images_to_npy']['topic']
        output_folder: str = self.operation_params['extract_images_to_npy']['output_folder']
        ImageData.from_ros2_bag(self.input_bag, topic, output_folder)

    def convert_ros2_to_ros1(self):
        """ Convert a ROS2 bag from the bag wrapper into a ROS1 bag. """

        self.bag_wrapper.export_as_ros1(self.output_bag, self.external_msgs_path_ros1)