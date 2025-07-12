from ..conversion_utils import convert_collection_into_decimal_array
from .Data import Data
import decimal
from decimal import Decimal
import numpy as np
from pathlib import Path
from rosbags.typesys import Stores, get_typestore
from typeguard import typechecked

class ImuData(Data):

    # Define IMU-specific data attributes
    lin_acc: np.ndarray[Decimal]
    ang_vel: np.ndarray[Decimal]
    orientation: np.ndarray[Decimal]

    @typechecked
    def __init__(self, frame_id: str, timestamps: np.ndarray | list, 
                 lin_acc: np.ndarray | list, ang_vel: np.ndarray | list,
                 orientation: np.ndarray | list):
        
        # Copy initial values into attributes
        super().__init__(frame_id, timestamps)
        self.lin_acc = convert_collection_into_decimal_array(lin_acc)
        self.ang_vel = convert_collection_into_decimal_array(ang_vel)
        self.orientation = convert_collection_into_decimal_array(orientation)

        # Check to ensure that all arrays have same length
        if len(self.timestamps) != len(self.lin_acc) or len(self.lin_acc) != len(self.ang_vel) \
            or len(self.ang_vel) != len(self.orientation):
            raise ValueError("Lengths of timestamp, lin_acc, ang_vel, and orientation arrays are not equal!")

    # =========================================================================
    # ============================ Class Methods ============================== 
    # =========================================================================  
        
    @classmethod
    @typechecked
    def from_TartanAir(cls, folder_path: Path | str, frame_id: str):
        """
        Creates a class structure from the TartanAir dataset format, which includes
        various .txt files with IMU data.

        Args:
            folder_path (Path | str): Path to the folder containing the IMU data.
            frame_id (str): The frame where this IMU data was collected.
        Returns:
            ImageData: Instance of this class.
        """

        # Get paths to all necessary files
        ts_folder_path = Path(folder_path) / 'imu_time.npy'
        lin_acc_folder_path = Path(folder_path) / 'acc_nograv_body.npy'
        ang_vel_folder_path =  Path(folder_path) / 'gyro.npy'
        orientation_folder_path = Path(folder_path) / 'ori_global.npy'

        # Load the data
        timestamps = convert_collection_into_decimal_array(np.load(ts_folder_path))
        lin_acc = np.load(lin_acc_folder_path)
        ang_vel = np.load(ang_vel_folder_path)

        # Currently unsure of format of TartanAir Orientation data
        # (whether it's extrinsic or intrinsic euler rotations, etc.)
        # Thus, for now fill with zeros.
        orientation = np.zeros_like(ang_vel)

        # Create the ImuData class
        return cls(frame_id, timestamps, lin_acc, ang_vel, orientation)

    # =========================================================================
    # =========================== Conversion to ROS =========================== 
    # ========================================================================= 

    @typechecked
    @staticmethod
    def get_ros_msg_type() -> str:
        """ Return the __msgtype__ for an Imu msg. """
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        return typestore.types['sensor_msgs/msg/Imu'].__msgtype__

    @typechecked
    def get_ros_msg(self, i: int):
        """
        Gets an Image ROS2 Humble message corresponding to the image represented by index i.
        
        Args:
            i (int): The index of the image message to convert.
        Raises:
            ValueError: If i is outside the data bounds.
        """

        # Check to make sure index is within data bounds
        if i < 0 or i >= self.len():
            raise ValueError(f"Index {i} is out of bounds!")

        # Get ROS2 message classes
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        Imu = typestore.types['sensor_msgs/msg/Imu']
        Header = typestore.types['std_msgs/msg/Header']
        Time = typestore.types['builtin_interfaces/msg/Time']
        Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
        Vector3 = typestore.types['geometry_msgs/msg/Vector3']

        # Get the seconds and nanoseconds
        seconds = int(self.timestamps[i])
        nanoseconds = (self.timestamps[i] - self.timestamps[i].to_integral_value(rounding=decimal.ROUND_DOWN)) * Decimal("1e9").to_integral_value(decimal.ROUND_HALF_EVEN)

        # Write the data into the new msg
        return Imu(Header(stamp=Time(sec=int(seconds), 
                                     nanosec=int(nanoseconds)), 
                          frame_id=self.frame_id),
                    orientation=Quaternion(x=0,
                                           y=0,
                                           z=0,
                                           w=1), # Currently ignores data in orientation
                    orientation_covariance=np.zeros(9),
                    angular_velocity=Vector3(x=self.ang_vel[i][0],
                                             y=self.ang_vel[i][1],
                                             z=self.ang_vel[i][2]),
                    angular_velocity_covariance=np.zeros(9),
                    linear_acceleration=Vector3(x=self.lin_acc[i][0],
                                                y=self.lin_acc[i][1], 
                                                z=self.lin_acc[i][2]),
                    linear_acceleration_covariance=np.zeros(9))
                    