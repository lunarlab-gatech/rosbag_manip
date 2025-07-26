from __future__ import annotations

from ..conversion_utils import convert_collection_into_decimal_array
import csv
from .Data import CoordinateFrame, Data
import decimal
from decimal import Decimal
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from pathlib import Path
from ..rosbag.Ros2BagWrapper import Ros2BagWrapper
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.store import Typestore
from scipy.spatial.transform import Rotation as R
from typeguard import typechecked
import tqdm

class OdometryData(Data):

    # Define odometry-specific data attributes
    child_frame_id: str
    frame: CoordinateFrame
    positions: np.ndarray[Decimal] # meters (x, y, z)
    orientations: np.ndarray[Decimal] # quaternions (x, y, z, w)
    poses = [] # Saved nav_msgs/msg/Pose

    @typechecked
    def __init__(self, frame_id: str, child_frame_id: str, frame: CoordinateFrame, 
                 timestamps: np.ndarray | list, positions: np.ndarray | list, 
                 orientations: np.ndarray | list):
        
        # Copy initial values into attributes
        super().__init__(frame_id, timestamps)
        self.child_frame_id = child_frame_id
        self.frame = frame
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

                # NOTE: Doesn't support Twist information currently

                # Increment the count
                i += 1
                pbar.update(1)

        # Create an OdometryData class
        return cls(frame_id, child_frame_id, CoordinateFrame.FLU, timestamps_np, positions_np, orientations_np)
    
    @classmethod
    @typechecked
    def from_csv(cls, csv_path: Path | str, frame_id: str, child_frame_id: str, frame: CoordinateFrame):
        """
        Creates a class structure from a csv file, where the order of values
        in the files follows ['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'].

        Args:
            csv_path (Path | str): Path to the CSV file.
            frame_id (str): The frame that this odometry is relative to.
            child_frame_id (str): The frame whose pose is represented by this odometry.
            frame (CoordinateFrame): The coordinate system convention of this data.
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
        return cls(frame_id, child_frame_id, frame, timestamps_np, positions_np, orientations_np)
    
    @classmethod
    @typechecked
    def from_txt_file(cls, file_path: Path | str, frame_id: str, child_frame_id: str, frame: CoordinateFrame):
        """
        Creates a class structure from a text file, where the order of values
        in the files follows ['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'].

        Args:
            file_path (Path | str): Path to the file containing the odometry data.
            frame_id (str): The frame where this odometry is relative to.
            child_frame_id (str): The frame whose pose is represented by this odometry.
            frame (CoordinateFrame): The coordinate system convention of this data.
        Returns:
            OdometryData: Instance of this class.
        """
        
        # Count the number of lines in the file
        line_count = 0
        with open(str(file_path), 'r') as file:
            for _ in file: 
                line_count += 1

        # Setup arrays to hold data
        timestamps_np = np.zeros((line_count), dtype=object)
        positions_np = np.zeros((line_count, 3), dtype=object)
        orientations_np = np.zeros((line_count, 4), dtype=object)

        # Open the txt file and read in the data
        with open(str(file_path), 'r') as file:
            for i, line in enumerate(file):
                line_split = line.split(' ')
                timestamps_np[i] = line_split[0]
                positions_np[i] = line_split[1:4]
                orientations_np[i] =  line_split[5:8] + [line_split[4]]
        
        # Create an OdometryData class
        return cls(frame_id, child_frame_id, frame, timestamps_np, positions_np, orientations_np)
    
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

    def shift_to_start_at_identity(self):
        """
        Alter the positions and orientations based so that the first pose 
        starts at Identity.
        """

        # Get pose of first robot position w.r.t world
        R_o = R.from_quat(self.orientations[0]).as_matrix()
        T_o = np.expand_dims(self.positions[0], axis=1)
        
        # Calculate the inverse (pose of world w.r.t first robot location)
        R_inv = R_o.T

        # Rotate positions and orientations
        self.positions = (R_inv @ (self.positions.T - T_o).astype(float)).T
        for i in range(self.len()):
            self.orientations[i] = R.from_matrix((R_inv @ R.from_quat(self.orientations[i]).as_matrix())).as_quat()

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

    # =========================================================================
    # =========================== Frame Conversions =========================== 
    # ========================================================================= 
    def to_FLU_frame(self):
        # If we are already in the FLU frame, return
        if self.frame == CoordinateFrame.FLU:
            print("Data already in FLU coordinate frame, returning...")
            return

        # If in NED, run the conversion
        elif self.frame == CoordinateFrame.NED:
            # Define the rotation matrix
            R_NED = np.array([[1,  0,  0],
                              [0, -1,  0],
                              [0,  0, -1]])
            R_NED_Q = R.from_matrix(R_NED)

            # Do a change of basis
            self.positions = (R_NED @ self.positions.T).T
            for i in range(self.len()):
                self.orientations[i] = (R_NED_Q * R.from_quat(self.orientations[i]) * R_NED_Q.inv()).as_quat()

            # Update frame
            self.frame = CoordinateFrame.FLU

        # Otherwise, throw an error
        else:
            raise RuntimeError(f"OdometryData class is in an unexpected frame: {self.frame}!")

    # =========================================================================
    # =========================== Conversion to ROS =========================== 
    # ========================================================================= 

    @typechecked
    @staticmethod
    def get_ros_msg_type(msg_type: str = "Odometry") -> str:
        """ Return the __msgtype__ for an Imu msg. """
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        if msg_type == "Odometry":
            return typestore.types['nav_msgs/msg/Odometry'].__msgtype__
        elif msg_type == "Path":
            return typestore.types['nav_msgs/msg/Path'].__msgtype__
        else:
            raise ValueError(f"Unsupported msg_type for OdometryData: {msg_type}")
    
    @typechecked
    def extract_seconds_and_nanoseconds(self, i: int):
        seconds = int(self.timestamps[i])
        nanoseconds = (self.timestamps[i] - self.timestamps[i].to_integral_value(rounding=decimal.ROUND_DOWN)) \
                        * Decimal("1e9").to_integral_value(decimal.ROUND_HALF_EVEN)
        return seconds, nanoseconds
    
    def calculate_stamped_poses(self):
        """ Pre-calculates all stamped poses if they aren't calculated yet."""

        if len(self.poses) != self.len():
            # Get ROS2 message types
            typestore = get_typestore(Stores.ROS2_HUMBLE)
            Header = typestore.types['std_msgs/msg/Header']
            Time = typestore.types['builtin_interfaces/msg/Time']
            PoseStamped = typestore.types['geometry_msgs/msg/PoseStamped']
            Pose = typestore.types['geometry_msgs/msg/Pose']
            Point = typestore.types['geometry_msgs/msg/Point']
            Quaternion = typestore.types['geometry_msgs/msg/Quaternion']

            # Pre-calculate all the poses
            for i in range(self.len()):
                seconds, nanoseconds = self.extract_seconds_and_nanoseconds(i)
                self.poses.append(PoseStamped(Header(stamp=Time(sec=int(seconds), 
                                                                nanosec=int(nanoseconds)),
                                                    frame_id=self.frame_id),
                                            pose=Pose(position=Point(x=self.positions[i][0],
                                                                    y=self.positions[i][1],
                                                                    z=self.positions[i][2]),
                            orientation=Quaternion(x=self.orientations[i][0],
                                                    y=self.orientations[i][1],
                                                    z=self.orientations[i][2],
                                                    w=self.orientations[i][3]))))

    @typechecked
    def get_ros_msg(self, i: int, msg_type: str = "Odometry"):
        """
        Gets an Image ROS2 Humble message corresponding to the odometry in index i.
        
        Args:
            i (int): The index of the odometry data to convert.
        Raises:
            ValueError: If i is outside the data bounds.
        """

        # Calculate the Stamped Poses
        self.calculate_stamped_poses()

        # Make sure our data is in the FLU frame, otherwise throw an error
        if self.frame != CoordinateFrame.FLU:
            raise RuntimeError("Convert this Odometry Data to a FLU frame before writing to a ROS2 bag!")

        # Check to make sure index is within data bounds
        if i < 0 or i >= self.len():
            raise IndexError(f"Index {i} is out of bounds!")

        # Get ROS2 message classes
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        Odometry = typestore.types['nav_msgs/msg/Odometry']
        Header = typestore.types['std_msgs/msg/Header']
        Time = typestore.types['builtin_interfaces/msg/Time']
        PoseWithCovariance = typestore.types['geometry_msgs/msg/PoseWithCovariance']
        TwistWithCovariance = typestore.types['geometry_msgs/msg/TwistWithCovariance']
        Twist = typestore.types['geometry_msgs/msg/Twist']
        Vector3 = typestore.types['geometry_msgs/msg/Vector3']
        Path = typestore.types['nav_msgs/msg/Path']
        Pose = typestore.types['geometry_msgs/msg/Pose']
        Point = typestore.types['geometry_msgs/msg/Point']
        Quaternion = typestore.types['geometry_msgs/msg/Quaternion']

        # Write the data into the new msg
        if msg_type == "Odometry":
            seconds, nanoseconds = self.extract_seconds_and_nanoseconds(i)
            return Odometry(Header(stamp=Time(sec=int(seconds), 
                                              nanosec=int(nanoseconds)), 
                            frame_id=self.frame_id),
                            child_frame_id=self.child_frame_id,
                            pose=PoseWithCovariance(pose=Pose(position=Point(x=self.positions[i][0],
                                                                 y=self.positions[i][1],
                                                                 z=self.positions[i][2]),
                                                    orientation=Quaternion(x=self.orientations[i][0],
                                                                            y=self.orientations[i][1],
                                                                            z=self.orientations[i][2],
                                                                            w=self.orientations[i][3])),
                                                    covariance=np.zeros(36)),
                            twist=TwistWithCovariance(twist=Twist(linear=Vector3(x=0, # Currently doesn't support Twist
                                                                                y=0,
                                                                                z=0,),
                                                                angular=Vector3(x=0,
                                                                                y=0,
                                                                                z=0,)),
                                                    covariance=np.zeros(36)))
        elif msg_type == "Path":
            seconds, nanoseconds = self.extract_seconds_and_nanoseconds(i)
            return Path(Header(stamp=Time(sec=int(seconds), 
                                          nanosec=int(nanoseconds)),
                               frame_id=self.frame_id),
                               poses=self.poses[0:i+1:10])
        else:
            raise ValueError(f"Unsupported msg_type for OdometryData: {msg_type}")