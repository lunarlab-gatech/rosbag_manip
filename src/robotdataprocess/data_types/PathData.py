from ..conversion_utils import convert_collection_into_decimal_array
from .Data import Data
from decimal import Decimal
import numpy as np
from pathlib import Path
from ..rosbag.Ros2BagWrapper import Ros2BagWrapper
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys.store import Typestore
from typeguard import typechecked
import tqdm

class PathData(Data):

    positions: np.ndarray[Decimal] # meters (x, y, z)
    orientations: np.ndarray[Decimal] # quaternions (x, y, z, w)

    @typechecked
    def __init__(self, frame_id: str, timestamps: np.ndarray | list, 
                 positions: np.ndarray | list, orientations: np.ndarray | list):
        super().__init__(frame_id, timestamps)
        self.positions = convert_collection_into_decimal_array(positions)
        self.orientations = convert_collection_into_decimal_array(orientations)

    # =========================================================================
    # ============================ Class Methods ============================== 
    # =========================================================================  

    @classmethod
    @typechecked
    def from_ros2_bag(cls, bag_path: Path | str, odom_topic: str):
        """
        Creates a class structure from a ROS2 bag file with a Path topic.

        Args:
            bag_path (Path | str): Path to the ROS2 bag file.
            odom_topic (str): Topic of the Path messages.
        Returns:
            PathData: Instance of this class.
        """

        # Get topic message count and typestore
        bag_wrapper = Ros2BagWrapper(bag_path, None)
        typestore: Typestore = bag_wrapper.get_typestore()
        num_msgs: int = bag_wrapper.get_topic_count(odom_topic)
        
        # Make empty arrays
        timestamps_np = np.zeros(0, dtype=Decimal)
        positions_np = np.zeros((0, 3), dtype=Decimal)
        orientations_np = np.zeros((0, 4), dtype=Decimal)

        # Setup tqdm bar & counter
        pbar = tqdm.tqdm(total=num_msgs, desc="Extracting Path...", unit=" msgs")

        # Extract the odometry information
        frame_id = None
        with Reader2(str(bag_path)) as reader:

            # Extract frame_id from first message
            connections = [x for x in reader.connections if x.topic == odom_topic]
            for conn, timestamp, rawdata in reader.messages(connections=connections):  
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                frame_id = msg.header.frame_id
                break

            # Extract message data
            connections = [x for x in reader.connections if x.topic == odom_topic]
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                
                # NOTE: Currently, this method doesn't track when each Path message 
                # is recieved, and throws away duplicate poses contained in multiple
                # Path messages.

                # Iterate through each pose in the message
                for pose in msg.poses:
                    
                    # See if we already have this pose (via timestamp)
                    ts = bag_wrapper.extract_timestamp(pose)
                    if ts in timestamps_np:
                        continue

                    # If not, extract data
                    timestamps_np = np.concatenate((timestamps_np, [ts]), axis= 0)
                    pos = pose.pose.position
                    positions_np = np.concatenate((positions_np, [[Decimal(pos.x), Decimal(pos.y), Decimal(pos.z)]]), axis=0)
                    ori = pose.pose.orientation
                    orientations_np = np.concatenate((orientations_np, [[Decimal(ori.x), Decimal(ori.y), Decimal(ori.z), Decimal(ori.w)]]), axis=0)

                    # Increment the count
                    pbar.update(1)

        # Create an OdometryData class
        return cls(frame_id, timestamps_np, positions_np, orientations_np)
    