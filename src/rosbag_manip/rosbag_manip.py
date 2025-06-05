from collections import defaultdict
from decimal import Decimal
import glob
import inspect
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from rosbags.rosbag1 import Writer as Writer1
from rosbags.rosbag1.writer import Connection
from rosbags.rosbag2 import Reader as Reader2
from rosbags.rosbag2 import Writer as Writer2
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.typesys.store import Typestore
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
        for key, value in kwargs.items():
            setattr(self, key, value)
        self.create_typestores_with_external_msgs()
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

    def create_typestores_with_external_msgs(self) -> None:
        """
        Create TypeStores with external message types added from the specified path.

        Used Attributes:
            self.external_msgs_path_ros2 (str): Path to the directory containing external message definitions for ROS2.
            self.external_msgs_path_ros1 (str): Path to the directory containing external message definitions for ROS1.
                Currently unused.

        Set Attributes:
            self.typestore1 (Typestore): The ROS1 Typestore instance containing the message types.
            self.typestore2 (Typestore): The ROS2 Typestore instance containing the message types.
        """

        def register_msgs(path: Path, typestore: Typestore):
            for msg_path in glob.glob(str(path / Path('**') /Path('*.msg')), recursive=True):
                msg_path = Path(msg_path)
                msgdef = msg_path.read_text(encoding='utf-8')
                typestore.register(get_types_from_msg(msgdef, rosbag_manipulation.guess_msgtype(msg_path)))

        self.typestore1 = get_typestore(Stores.ROS1_NOETIC)
        self.typestore2 = get_typestore(Stores.ROS2_HUMBLE)
        
        # Load external message types
        try: register_msgs(Path(self.external_msgs_path_ros2), self.typestore2)
        except: print("No `external_msgs_path_ros2` path provided, using no external msgs for ros2 Typestore")
        try: register_msgs(Path(self.external_msgs_path_ros1), self.typestore1)
        except: print("No `external_msgs_path_ros1` path provided, using no external msgs for ros1 Typestore")

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
                msg = self.typestore2.deserialize_cdr(rawdata, conn.msgtype)

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
    # ============================= Downsampling ==============================
    # ========================================================================= 

    def downsample(self):
        """
        Downsample a ROS2 bag file by downsampling the frequency of specified topics. Additionally
        includes or excludes unmentioned topics based on `include_unmentioned_topics` parameter, which
        can be used to prune topics.

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
        with Reader2(input_bag) as reader:
            connections = reader.connections
            
            # Create a writer with only the specified topics
            with Writer2(output_bag, version=5) as writer:
                conn_map = {}
                for conn in connections:
                    if conn.topic in topic_downsample_ratios:
                        conn_map[conn.topic] = writer.add_connection(
                            topic=conn.topic,
                            msgtype=conn.msgtype,
                            msgdef=conn.msgdef,
                            typestore=self.typestore2,
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
                        typestore=self.typestore2,
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
    # ============================= Conversions ===============================
    # ========================================================================= 

    # Defines msg_type mappings from ROS2 to ROS1
    msg_mapping_ros2_to_ros1 = {
        "builtin_interfaces/msg/Time": "builtin_interfaces/msg/Time",
        "geometry_msgs/msg/Point": "geometry_msgs/msg/Point",
        "geometry_msgs/msg/Pose": "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/PoseWithCovariance": "geometry_msgs/msg/PoseWithCovariance",
        "geometry_msgs/msg/Quaternion": "geometry_msgs/msg/Quaternion",
        "geometry_msgs/msg/Twist": "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistWithCovariance": "geometry_msgs/msg/TwistWithCovariance",
        "geometry_msgs/msg/Vector3": "geometry_msgs/msg/Vector3",
        "nav_msgs/msg/Odometry": "nav_msgs/msg/Odometry",
        "sensor_msgs/msg/Image": "sensor_msgs/msg/Image",
        "sensor_msgs/msg/CameraInfo": "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/RegionOfInterest": "sensor_msgs/msg/RegionOfInterest",
        "std_msgs/msg/Header": "std_msgs/msg/Header",
    }

    def get_mapping(self, msg_type: str) -> dict:
        """
        Given the msg_type string, return a mapping dictionary that describes how to map 
        values from a ros2 message to a ros1 message. Only supports a limited set of 
        message types currently, but can be extended to support more.

        Args:
            msg_type (str): The message type string, e.g., "sensor_msgs/msg/Image".
        Returns:
            dict: A mapping dictionary where:
                keys (str): A string defining the attribute in the ros2 message.
                    Thus, a key of "header.stamp" would map to ros2_msg.header.stamp.
                values (str or tuple): A string defining the attribute in the ros1 message,
                    OR a tuple where the first element is the attribute in the ros1 message and
                    the second element is a mapping dictionary for mapping attributes within 
                    child message types.
        """

        map = None
        match msg_type:
            case "builtin_interfaces/msg/Time":
                map = {
                    "sec": "sec",
                    "nanosec": "nanosec"
                }
            case "geometry_msgs/msg/Point":
                map = {
                    "x": "x",
                    "y": "y",
                    "z": "z"
                }
            case "geometry_msgs/msg/Pose":
                map = {
                    "position": ("position", self.get_mapping("geometry_msgs/msg/Point")),
                    "orientation": ("orientation", self.get_mapping("geometry_msgs/msg/Quaternion"))
                }
            case "geometry_msgs/msg/PoseWithCovariance":
                map = {
                    "pose": ("pose", self.get_mapping("geometry_msgs/msg/Pose")),
                    "covariance": "covariance"
                }
            case "geometry_msgs/msg/Quaternion":
                map = {
                    "x": "x",
                    "y": "y",
                    "z": "z",
                    "w": "w"
                }
            case "geometry_msgs/msg/Twist":
                map = {
                    "linear": ("linear", self.get_mapping("geometry_msgs/msg/Vector3")),
                    "angular": ("angular", self.get_mapping("geometry_msgs/msg/Vector3"))
                }
            case "geometry_msgs/msg/TwistWithCovariance":
                map = {
                    "twist": ("twist", self.get_mapping("geometry_msgs/msg/Twist")),
                    "covariance": "covariance"          
                }
            case "geometry_msgs/msg/Vector3":
                map = {
                    "x": "x",
                    "y": "y",
                    "z": "z"
                }
            case "nav_msgs/msg/Odometry":
                map = {
                    "header": ("header", self.get_mapping("std_msgs/msg/Header")),
                    "child_frame_id": "child_frame_id",
                    "pose": ("pose", self.get_mapping("geometry_msgs/msg/PoseWithCovariance")),
                    "twist": ("twist", self.get_mapping("geometry_msgs/msg/TwistWithCovariance"))
                }
            case "sensor_msgs/msg/Image":
                map = {
                    "header": ("header", self.get_mapping("std_msgs/msg/Header")),
                    "height": "height",
                    "width": "width",
                    "encoding": "encoding", # Not, this may not map 1-to-1 if encoding strings are different between ROS1/ROS2
                    "is_bigendian": "is_bigendian",
                    "step": "step",
                    "data": "data"
                }
            case "sensor_msgs/msg/CameraInfo":
                map = {
                    "header": ("header", self.get_mapping("std_msgs/msg/Header")),
                    "height": "height",
                    "width": "width",
                    "distortion_model": "distortion_model", # May not map 1-to-1 if models differ between ROS1/ROS2
                    "d": "D",
                    "k": "K",
                    "r": "R",
                    "p": "P",
                    "binning_x": "binning_x",
                    "binning_y": "binning_y",
                    "roi": ("roi", self.get_mapping("sensor_msgs/msg/RegionOfInterest"))
                }
            case "sensor_msgs/msg/RegionOfInterest":
                map = {
                    "x_offset": "x_offset",
                    "y_offset": "y_offset",
                    "height": "height",
                    "width": "width",
                    "do_rectify": "do_rectify"
                }
            case "std_msgs/msg/Header":
                map = {
                    "stamp": ("stamp", self.get_mapping("builtin_interfaces/msg/Time")),
                    "frame_id": "frame_id"
                    # Note: ROS1 msg has a 'seq' value, which we define no mapping for.
                    # Thus, default value in self.get_ros1_msg_instance is used.
                }
            case _:
                raise ValueError(f"Currently unsupported msg_type: {msg_type}")
            
        return map
        
        
    def solve_for_all_mappings(self, map: dict[str, str | tuple] | None, msg_type: str | None) -> dict:
        """
        Helper method that will take the provided map and recursively solve for all mappings
        that are not yet resolved. Additionally, if a map isn't provided, it will generate
        an initial map from the provided msg_type.

        Args:
            map (dict | None): A mapping dictionary where:
                keys (list of str): A list defining the attribute in the ros2 message.
                    Thus, a key of "header.stamp" would map to ros2_msg.header.stamp.
                values (list of str or tuple): A list defining the attribute in the ros1 message,
                    OR a tuple where the first element is the attribute in the ros1 message and
                    the second element is a mapping dictionary for mapping attributes within 
                    child message types.
              If None, then 'msg_type' is required to generate an initial map.
            msg_type (str | None): The message type string, e.g., "sensor_msgs/msg/Image".
                If map is None, an initial map will be generated from the msg_type.
        
        Returns:
            dict: A mapping dictionary where:
                keys (list of str): A list defining the attribute in the ros2 message.
                    Thus, a key of "header.stamp" would map to ros2_msg.header.stamp.
                values (list of str): A list defining the attribute in the ros1 message. Note
                that this can no longer be a tuple, as all mappings have been resolved.
        """
        
        # Check input parameters
        if map is None and msg_type is not None:
            # If no map is provided, we need to generate it from the msg_type
            map = self.get_mapping(msg_type)
        elif map is None and msg_type is None:
            raise ValueError("No map provided and no msg_type specified to generate initial map.")
        elif map is not None and msg_type is not None:
            print("WARNING: msg_type provided, but map is already defined. Using provided map and ignoring msg_type.")

        # Iterate through each key in the map to check for unresolved mappings
        for key in map.copy():
            # There are unresolved mappings, so we need to solve them
            if type(map[key]) is tuple:

                # Extract value and child map
                value = map[key][0]
                child_map = map[key][1]

                # Solve recursively to only get values with type "list" in the child map
                child_map = self.solve_for_all_mappings(child_map, None)


                # Now, take child map and bring it into the current map
                for child_key in child_map:
                    new_key = '.'.join(key.split('.') + child_key.split('.'))
                    new_value = '.'.join(value.split('.') + child_map[child_key].split('.'))
                    map[new_key] = new_value
                
                # Remove the old key
                del map[key]
        return map
    
    def get_ros1_msg_instance(self, msg_type: str) -> object:
        """
        Get a ROS1 message instance with all values filled with defaults.
        This allows us to easily add attributes later on, instead of needing
        all of the attributes ready to go before initialization.

        Args:
            msg_type (str): The type of message to initialize a ROS1 msg class for.

        Returns:
            object: The initialized ROS1 class message.
        """

        # Initialize the ros1_msg class
        ros1_msg_class = self.typestore1.types[self.msg_mapping_ros2_to_ros1[msg_type]]

        # Get the signature of the __init__ method
        sig = inspect.signature(ros1_msg_class.__init__)

        # Set default values based on parameter type
        initial_parameters = []
        for name, param in sig.parameters.items():
            type_str = str(param.annotation)
            if type_str == "<class 'inspect._empty'>" or type_str == 'ClassVar[str]':
                pass
            elif type_str == "int":
                initial_parameters.append(0)
            elif type_str == "str":
                initial_parameters.append("")
            elif type_str == "bool":
                initial_parameters.append(False)
            elif type_str == "float":
                initial_parameters.append(0.0)
            elif "np.ndarray" in type_str:
                initial_parameters.append(np.array([]))
            elif "msg" in type_str:
                initial_parameters.append(self.get_ros1_msg_instance("/".join(type_str.split('__'))))
            else:
                raise NotImplementedError(f"Unspecified default value: {type_str}")

        # Initialize class with dummy values
        return ros1_msg_class(*initial_parameters)


    def map_ros2_to_ros1(self, ros2_msg: object, msg_type: str) -> object:
        """
        Map a ROS2 message to a ROS1 message based on the provided message type.

        Args:
            ros2_msg (object): The ROS2 message object to be mapped.
            msg_type (str): The message type string, e.g., "sensor_msgs/msg/Image".

        Returns:
            object: The mapped ROS1 message object.
        """

        # Get ros1 message class
        ros1_msg = self.get_ros1_msg_instance(msg_type)
        
        # Map attributes from ros2_msg to ros1_msg
        for ros2_attr_str, ros1_attr_str in self.solve_for_all_mappings(None, msg_type).items():
            ros2_attr_list = ros2_attr_str.split('.')
            ros1_attr_list = ros1_attr_str.split('.')

            # Map each message to temporary objects
            ros2_obj = ros2_msg
            ros1_obj = ros1_msg

            # Iterate down to the base attribute in each message
            while len(ros2_attr_list) > 1:
                ros2_obj = getattr(ros2_obj, ros2_attr_list[0])
                ros2_attr_list = ros2_attr_list[1:]

            while len(ros1_attr_list) > 1:
                ros1_obj = getattr(ros1_obj, ros1_attr_list[0])
                ros1_attr_list = ros1_attr_list[1:]

            # Make sure the attribute we're about to set already has a value
            if hasattr(ros1_obj, ros1_attr_list[0]):
                setattr(ros1_obj, ros1_attr_list[0], getattr(ros2_obj, ros2_attr_list[0]))
            else:
                raise Exception(f"Attribute '{ros1_attr_list[0]}' to set doesn't exist! Double check values in self.get_mapping().")
        
        return ros1_msg

    def convert_ros2_to_ros1(self):
        """
        Convert a ROS2 bag file into a ROS1 bag file.

        Used Attributes:
            self.input_bag (str): Path to the input ROS2 bag file.
            self.output_bag (str): Path to the output ROS1 bag file.
        """

        # Convert to pathlib paths
        input_bag = Path(self.input_bag)
        output_bag = Path(self.output_bag)

        # Make sure we aren't overwriting an existing output bag
        if output_bag.exists():
            raise AssertionError("Delete Output Directory first!")

        with Reader2(input_bag) as reader:
            connections = reader.connections
            
            # Setup the bag writer
            with Writer1(output_bag) as writer:
                conn_map = {}
                for conn in connections:
                    # Determine latching behavior (if possible)
                    latching = None
                    if len(conn.ext.offered_qos_profiles) == 1:
                        if conn.ext.offered_qos_profiles[0].durability.value == 2: latching = 0 # VOLATILE
                        elif conn.ext.offered_qos_profiles[0].durability.value == 1: latching = 1 # TRANSIENT_LOCAL
                    else: pass

                    # Only write supported types
                    msg_def = None
                    try: 
                        msg_def = self.typestore1.types[self.msg_mapping_ros2_to_ros1[conn.msgtype]]
                    except:
                        print(f"No mapping currently provided for {conn.msgtype}, skipping...")
                        continue

                    # Add the connection
                    new_conn: Connection = writer.add_connection(
                        topic=conn.topic,
                        msgtype=self.msg_mapping_ros2_to_ros1[conn.msgtype],
                        msgdef=msg_def,
                        typestore=self.typestore1,
                        md5sum=None,
                        callerid=None,
                        latching=latching
                    )
                    conn_map[conn.topic] = new_conn

                # setup tqdm 
                pbarW = tqdm.tqdm(total=None, desc="Writing messages", unit=" messages")

                # Iternate through messages in the bag
                for conn, timestamp, rawdata in reader.messages():
                    # Only writes messages that had a connection successfully created
                    if conn.topic in list(conn_map.keys()):
                        # Deserialize the ROS2 message
                        ros2_msg = self.typestore2.deserialize_cdr(rawdata, conn.msgtype)

                        # Map the ROS2 message to a ROS1 message
                        ros1_msg = self.map_ros2_to_ros1(ros2_msg, conn.msgtype)

                        # Serialize the ROS1 message
                        rawdata = self.typestore1.serialize_ros1(ros1_msg, conn_map[conn.topic].msgtype)

                        # Write the message to the ROS1 bag
                        writer.write(conn_map[conn.topic], timestamp, rawdata)
                        pbarW.update(1)

