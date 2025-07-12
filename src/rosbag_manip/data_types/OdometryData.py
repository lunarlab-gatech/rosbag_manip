from __future__ import annotations

from ..conversion_utils import convert_collection_into_decimal_array
import csv
from .Data import Data
from decimal import Decimal
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from pathlib import Path
from ..rosbag.Ros2BagWrapper import Ros2BagWrapper
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys.store import Typestore
from scipy.spatial.transform import Rotation as R
from typeguard import typechecked
import tqdm

class OdometryData(Data):

    # Define odometry-specific data attributes
    child_frame_id: str
    positions: np.ndarray[Decimal] # meters (x, y, z)
    orientations: np.ndarray[Decimal] # quaternions (x, y, z, w)

    @typechecked
    def __init__(self, frame_id: str, child_frame_id: str, timestamps: np.ndarray | list, 
                 positions: np.ndarray | list, orientations: np.ndarray | list):
        
        # Copy initial values into attributes
        super().__init__(frame_id, timestamps)
        self.child_frame_id = child_frame_id
        self.positions = convert_collection_into_decimal_array(positions)
        self.orientations = convert_collection_into_decimal_array(orientations)

        # Check to ensure that all arrays have same length
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
                break

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
    
    @classmethod
    @typechecked
    def from_csv(cls, csv_path: Path | str, frame_id: str, child_frame_id: str):
        """
        Creates a class structure from a csv file, where the order of values
        in the files follows ['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'].

        Args:
            csv_path (Path | str): Path to the CSV file.
            frame_id (str): The frame that this odometry is relative to.
            child_frame_id (str): The frame whose pose is represented by this odometry.
        Returns:
            OdometryData: Instance of this class.
        """

        # Read the csv file
        df1 = pd.read_csv(str(csv_path), header=0, names=['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
        
        # Convert columns to np.ndarray[Decimal]
        timestamps_np = np.array([Decimal(str(ts)) for ts in df1['timestamp']], dtype=object)
        positions_np = np.array([[Decimal(str(x)), Decimal(str(y)), Decimal(str(z))] 
                                 for x, y, z in zip(df1['x'], df1['y'], df1['z'])], dtype=object)
        orientations_np = np.array([[Decimal(str(qx)), Decimal(str(qy)), Decimal(str(qz)), Decimal(str(qw))]
                                    for qx, qy, qz, qw in zip(df1['qx'], df1['qy'], df1['qz'], df1['qw'])], dtype=object)

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
            self.positions[i][0] += Decimal(cumulative_noise_pos['x'])
            self.positions[i][1] += Decimal(cumulative_noise_pos['y'])
            self.positions[i][2] += Decimal(cumulative_noise_pos['z'])

    def shift_position(self, x_shift: float, y_shift: float, z_shift: float):
        """
        Shifts the positions of the odometry.

        Args:
            x_shift (float): Shift in x-axis.
            y_shift (float): Shift in y_axis.
            z_shift (float): Shift in z_axis.
        """
        self.positions[:,0] += Decimal(x_shift)
        self.positions[:,1] += Decimal(y_shift)
        self.positions[:,2] += Decimal(z_shift)

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

    # =========================================================================
    # ============================ Visualization ============================== 
    # =========================================================================  

    @typechecked
    def visualize(self, otherList: list[OdometryData], titles: list[str]):
        """
        Visualizes this OdometryData (and all others included in otherList)
        on a single plot.

        Args:
            otherList (list[OdometryData]): All other OdometryData objects whose
                odometry should also be visualized on this plot.
            titles (list[str]): Titles for each OdometryData object, starting 
                with self.
        """

        def draw_axes(data: OdometryData, label_prefix=""):
            """Helper function that visualizes orientation along the trajectory path with axes."""
            axes_interval = 5000
            axes_length = 5

            for i in range(0, data.len(), axes_interval):
                # Extract data
                pos = data.positions[i].astype(np.float64)
                quat = data.orientations[i].astype(np.float64)
                rot = R.from_quat(quat, scalar_first=False)

                # Define unit vectors for X, Y, Z in local frame
                x_axis = rot.apply([1, 0, 0])
                y_axis = rot.apply([0, 1, 0])
                z_axis = rot.apply([0, 0, 1])

                # Plot axes
                ax.quiver(*pos, *x_axis, length=axes_length, color='r', normalize=True, linewidth=0.8)
                ax.quiver(*pos, *y_axis, length=axes_length, color='g', normalize=True, linewidth=0.8)
                ax.quiver(*pos, *z_axis, length=axes_length, color='b', normalize=True, linewidth=0.8)

        # Ensure that the lists are of the proper sizes
        if (len(otherList) + 1) != len(titles):
            raise ValueError("Length of titles should be one more than length of otherlist!")

        # Build a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(self.positions[:,0].astype(np.float64), 
                self.positions[:,1].astype(np.float64), 
                self.positions[:,2].astype(np.float64), label=titles[0])
        for i, other in enumerate(otherList):
            ax.plot(other.positions[:,0].astype(np.float64), 
                    other.positions[:,1].astype(np.float64), 
                    other.positions[:,2].astype(np.float64), 
                    label=titles[1+i])

        # Draw orientation axes (X = red, Y = green, Z = blue)
        draw_axes(self)
        for other in otherList:
            draw_axes(other)

        # Set labels
        ax.set_title("Trajectory Comparison with Full Orientation")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend()

        # Concatenate all x, y and z values together
        all_x = self.positions[:,0]
        all_y = self.positions[:,1]
        all_z = self.positions[:,2]
        for other in otherList:
            all_x = np.concatenate((all_x, other.positions[:,0]))
            all_y = np.concatenate((all_y, other.positions[:,1]))
            all_z = np.concatenate((all_z, other.positions[:,2]))
        all_x = all_x.astype(np.float64)
        all_y = all_y.astype(np.float64)
        all_z = all_z.astype(np.float64)

        # Set an equal scale for all axes
        x_center = (all_x.max() + all_x.min()) / 2
        y_center = (all_y.max() + all_y.min()) / 2
        z_center = (all_z.max() + all_z.min()) / 2
        max_range = max(all_x.max() - all_x.min(), all_y.max() - all_y.min(), all_z.max() - all_z.min()) / 2
        ax.set_xlim(x_center - max_range, x_center + max_range)
        ax.set_ylim(y_center - max_range, y_center + max_range)
        ax.set_zlim(z_center - max_range, z_center + max_range)

        # Show the plot
        plt.tight_layout()
        plt.show()