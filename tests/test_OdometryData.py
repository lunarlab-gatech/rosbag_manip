import numpy as np
import os
from pathlib import Path
from robotdataprocess import CoordinateFrame
from robotdataprocess.data_types.OdometryData import OdometryData
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper
import unittest

class TestOdometryData(unittest.TestCase):
    
    def test_from_txt_file(self):
        """
        Test that we can load Odometry data from a txt file 
        and save it into a ROS2 bag.
        """

        # Load the Odometry data
        file_path = Path(Path('.'), 'tests', 'test_outputs', 'test_from_txt_file', 'odom.txt').absolute()
        odom_data = OdometryData.from_txt_file(file_path, '/Husky1', '/Husky1/base_link', CoordinateFrame.ROS)
        bag_path = Path(Path('.'), 'tests', 'test_bags', 'test_from_txt_file', 'odom_bag').absolute()
        if os.path.isdir(bag_path):
            os.remove(bag_path / 'odom_bag.db3')
            os.remove(bag_path / 'metadata.yaml')
            os.rmdir(bag_path)

        # Save it into a ROS2 bag
        Ros2BagWrapper.write_data_to_rosbag(bag_path, [odom_data], ['/odom'], [None], None)

        # Load the data back again
        ros_data = OdometryData.from_ros2_bag(bag_path, '/odom')

        # Make sure this data matches what we expect
        np.testing.assert_equal(float(ros_data.timestamps[13801]), 690.100000)
        np.testing.assert_array_equal(ros_data.positions[13801].astype(np.float128), [-66.153381, -76.155663, 1.445448])
        np.testing.assert_array_equal(ros_data.orientations[13801].astype(np.float128), [0.399908, 0.001246, -0.000566, 0.916554])
        np.testing.assert_equal(ros_data.frame_id, '/Husky1')
        np.testing.assert_equal(ros_data.child_frame_id, '/Husky1/base_link')
        np.testing.assert_equal(ros_data.frame, CoordinateFrame.ROS)

    def test_to_ROS_frame(self):
        """ 
        Makes sure that the conversion from NED to ROS functions properly.
        """

        # Load the Odometry data
        file_path = Path(Path('.'), 'tests', 'test_outputs', 'test_from_txt_file', 'odom.txt').absolute()
        odom_data = OdometryData.from_txt_file(file_path, '/Husky1', '/Husky1/base_link', CoordinateFrame.NED)

        # Converts it into the ROS coordinate system
        odom_data.to_ROS_frame()

        # Make sure this data matches what we expect
        np.testing.assert_equal(float(odom_data.timestamps[13801]), 690.100000)
        np.testing.assert_array_equal(odom_data.positions[13801].astype(np.float128), [-66.153381, 76.155663, -1.445448])
        np.testing.assert_array_almost_equal(odom_data.orientations[13801].astype(np.float128), [0.9165542249453513, 0.0005660057901561427, 0.0012459982892449885, -0.3999080892661899], 8)
        np.testing.assert_equal(odom_data.frame_id, '/Husky1')
        np.testing.assert_equal(odom_data.child_frame_id, '/Husky1/base_link')
        np.testing.assert_equal(odom_data.frame, CoordinateFrame.ROS)

if __name__ == "__main__":
    unittest.main()