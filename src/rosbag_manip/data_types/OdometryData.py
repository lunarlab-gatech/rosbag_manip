from ..conversion_utils import convert_collection_into_decimal_array
import csv
from decimal import Decimal
import numpy as np
import os
from pathlib import Path
from ..rosbag.Ros2BagWrapper import Ros2BagWrapper
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys.store import Typestore
from typeguard import typechecked
import tqdm

class OdometryData:

    # Define data attributes
    frame_id: str
    child_frame_id: str
    timestamps: np.ndarray[Decimal] # seconds
    positions: np.ndarray[Decimal] # meters (x, y, z)
    orientations: np.ndarray[Decimal] # quaternions (x, y, z, w)

    @typechecked
    def __init__(self, frame_id: str, child_frame_id: str, timestamps: np.ndarray | list, 
                 positions: np.ndarray | list, orientations: np.ndarray | list):
        
        # Copy initial values into attributes
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.timestamps = convert_collection_into_decimal_array(timestamps)
        self.positions = convert_collection_into_decimal_array(positions)
        self.orientations = convert_collection_into_decimal_array(orientations)

        # Check to ensure that all timestamps are sequential & arrays have same length
        for i in range(len(self.timestamps) - 1):
            if self.timestamps[i] >= self.timestamps[i+1]:
                raise ValueError("Timestamps do not come in sequential order!")
            
        if len(self.timestamps) != len(self.positions) or len(self.positions) != len(self.orientations):
            raise ValueError("Lengths of timestamp, position, and orientation arrays are not equal!")

    # =========================================================================
    # ============================ Class Methods ============================== 
    # =========================================================================  

    @classmethod
    @typechecked
    def from_ros2_bag(cls, bag_path: Path | str, odom_topic: str):
        """
        Creates a class structure from a ROS2 bag file with an Odometry topic.

        Args:
            bag_path (Path | str): Path to the ROS2 bag file.
            odom_topic (str): Topic of the Odometry messages.
        Returns:
            OdometryData: Instance of this class.
        """

        # Get topic message count and typestore
        bag_wrapper = Ros2BagWrapper(bag_path, None)
        typestore: Typestore = bag_wrapper.get_typestore()
        num_msgs: int = bag_wrapper.get_topic_count(odom_topic)
        
        # Pre-allocate arrays (memory-mapped or otherwise)
        timestamps_np = np.zeros(num_msgs, dtype=Decimal)
        positions_np = np.zeros((num_msgs, 3), dtype=Decimal)
        orientations_np = np.zeros((num_msgs, 4), dtype=Decimal)

        # Setup tqdm bar & counter
        pbar = tqdm.tqdm(total=num_msgs, desc="Extracting Odometry...", unit=" msgs")
        i = 0

        # Extract the odometry information
        frame_id, child_frame_id = None, None
        with Reader2(str(bag_path)) as reader:

            # Extract frames from first message
            connections = [x for x in reader.connections if x.topic == odom_topic]
            for conn, timestamp, rawdata in reader.messages(connections=connections):  
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                frame_id = msg.header.frame_id
                child_frame_id = msg.child_frame_id


            # Extract individual message data
            connections = [x for x in reader.connections if x.topic == odom_topic]
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                
                timestamps_np[i] = bag_wrapper.extract_timestamp(msg)

                pos = msg.pose.pose.position
                positions_np[i] = np.array([Decimal(pos.x), Decimal(pos.y), Decimal(pos.z)])

                ori = msg.pose.pose.orientation
                orientations_np[i] = np.array([Decimal(ori.x), Decimal(ori.y), Decimal(ori.z), Decimal(ori.w)])

                # Increment the count
                i += 1
                pbar.update(1)

        # Create an OdometryData class
        return cls(frame_id, child_frame_id, timestamps_np, positions_np, orientations_np)
    
    # =========================================================================
    # ========================= Manipulation Methods ========================== 
    # =========================================================================  
    
    def add_folded_guassian_noise_to_position(self, xy_noise_std_per_frame: float,
            z_noise_std_per_frame: float):
        """
        This method simulates odometry drift by adding folded gaussian noise
        to the odometry positions on a per frame basis. It also accumulates
        it over time. NOTE: It completely ignores the timestamps, and the "folded
        guassian noise" distribution stds might not align with the stds of the 
        guassian used internally, so this is not a robust function at all.

        Args:
            xy_noise_std_per_frame (float): Standard deviation of the gaussian 
                distribution for xy, whose output is then run through abs().
            z_noise_std_per_frame (float): Same as above, but for z.
        """

        # Track cumulative noise for each field
        cumulative_noise_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # For each position
        for i in range(len(self.timestamps)):

            # Sample noise and accumulate
            noise_pos = {'x': np.random.normal(0, xy_noise_std_per_frame),
                        'y': np.random.normal(0, xy_noise_std_per_frame),
                        'z': np.random.normal(0, z_noise_std_per_frame)}
            for key in cumulative_noise_pos:
                cumulative_noise_pos[key] += abs(noise_pos[key])

            # Update positions
            self.positions[i][0] += cumulative_noise_pos['x']
            self.positions[i][1] += cumulative_noise_pos['y']
            self.positions[i][2] += cumulative_noise_pos['z']

    # =========================================================================
    # ============================ Export Methods ============================= 
    # =========================================================================  

    def to_csv(self, csv_path: Path | str):
        """
        Writes the odometry data to a .csv file. Note that data will be
        saved in the following order: timestamp, pos.x, pos.y, pos.z,
        ori.w, ori.x, ori.y, ori.z. Timestamp is in seconds.

        Args:
            csv_path (Path | str): Path to the output csv file.
            odom_topic (str): Topic of the Odometry messages.
        Returns:
            OdometryData: Instance of this class.
        """

        # setup tqdm 
        pbar = tqdm.tqdm(total=None, desc="Saving to csv... ", unit=" frames")

        # Check that file path doesn't already exist
        file_path = Path(csv_path)
        if os.path.exists(file_path):
            raise ValueError(f"Output file already exists: {file_path}")
        
        # Open csv file
        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')

            # Write the first row
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
                
            # Write message data to the csv file
            for i in range(len(self.timestamps)):
                writer.writerow([str(self.timestamps[i]), 
                    str(self.positions[i][0]), str(self.positions[i][1]), str(self.positions[i][2]),
                    str(self.orientations[i][3]), str(self.orientations[i][0]), str(self.orientations[i][1]), 
                    str(self.orientations[i][2])])
                pbar.update(1)


