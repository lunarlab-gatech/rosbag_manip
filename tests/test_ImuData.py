import numpy as np
import os
from pathlib import Path
from rosbag_manip import CoordinateFrame
from rosbag_manip.data_types.ImuData import ImuData
from rosbag_manip.rosbag.Ros2BagWrapper import Ros2BagWrapper
import unittest

class TestImuData(unittest.TestCase):
    
    # TODO: Write test cases for: 
    # - Orientation loading for from_ros2_bag
    # - Orientation conversion from NED to ROS frame works as well.
    # These are technically written, but don't test on any orientation other than identity.
    
    def test_from_txt_file(self):
        """
        Test that we can load IMU data from a txt file 
        and save it into a ROS2 bag.
        """

        # Load the IMU data and save it into a ROS2 bag
        file_path = Path(Path('.'), 'tests', 'test_outputs', 'test_from_txt_file', 'imu.txt').absolute()
        imu_data = ImuData.from_txt_file(file_path, '/Husky1/base_link', CoordinateFrame.ROS)
        bag_path = Path(Path('.'), 'tests', 'test_bags', 'test_from_txt_file', 'imu_bag').absolute()
        if os.path.isdir(bag_path):
            os.remove(bag_path / 'imu_bag.db3')
            os.remove(bag_path / 'metadata.yaml')
            os.rmdir(bag_path)
        Ros2BagWrapper.write_data_to_rosbag(bag_path, [imu_data], ['/imu'], [None], None)

        # Load the data back again
        ros_data = ImuData.from_ros2_bag(bag_path, '/imu', '/Husky1/base_link')

        # Make sure this data matches what we expect
        np.testing.assert_equal(float(ros_data.timestamps[87212]), 436.065000)
        np.testing.assert_array_equal(ros_data.lin_acc[87212].astype(np.float128), [-0.124648, -0.091863, -10.415014])
        np.testing.assert_array_equal(ros_data.ang_vel[87212].astype(np.float128), [0.001785, 0.004928, 0.003135])
        np.testing.assert_array_equal(ros_data.orientation[87212].astype(np.float128), [0, 0, 0, 1])
        np.testing.assert_equal(ros_data.frame_id, '/Husky1/base_link')

    # def test_to_ROS_frame(self):
    #     """ 
    #     Makes sure that the conversion from NED to ROS functions properly.
    #     """

    #     # Load the IMU data
    #     file_path = Path(Path('.'), 'tests', 'test_outputs', 'test_from_txt_file', 'imu.txt').absolute()
    #     imu_data = ImuData.from_txt_file(file_path, '/Husky1/base_link', CoordinateFrame.NED)

    #     # Convert into a ROS frame
    #     imu_data.to_ROS_frame()

    #     # Make sure this data matches what we expect
    #     np.testing.assert_equal(float(imu_data.timestamps[87212]), 436.065000)
    #     np.testing.assert_array_equal(imu_data.lin_acc[87212].astype(np.float128), [-0.124648, 0.091863, 10.415014])
    #     np.testing.assert_array_equal(imu_data.ang_vel[87212].astype(np.float128), [0.001785, -0.004928, -0.003135])
    #     np.testing.assert_array_equal(imu_data.orientation[87212].astype(np.float128), [1, 0, 0, 0])
    #     np.testing.assert_equal(imu_data.frame_id, '/Husky1/base_link')

if __name__ == "__main__":
    unittest.main()