import numpy as np
import os
from pathlib import Path
from rosbag_manip.data_types.OdometryData import OdometryData
from rosbag_manip.rosbag.Ros2BagWrapper import Ros2BagWrapper
import unittest

class TestOdometryData(unittest.TestCase):
    
    def test_from_txt_file(self):
        """
        Test that we can load Odometry data from a txt file 
        and save it into a ROS2 bag.
        """

        # Load the Odometry data and save it into a ROS2 bag
        file_path = Path(Path('.'), 'tests', 'test_outputs', 'test_from_txt_file', 'odom.txt').absolute()
        odom_data = OdometryData.from_txt_file(file_path, '/Husky1', '/Husky1/base_link')
        bag_path = Path(Path('.'), 'tests', 'test_bags', 'test_from_txt_file', 'odom_bag').absolute()
        if os.path.isdir(bag_path):
            os.remove(bag_path / 'odom_bag.db3')
            os.remove(bag_path / 'metadata.yaml')
            os.rmdir(bag_path)
        Ros2BagWrapper.write_data_to_rosbag(bag_path, [odom_data], ['/odom'], [None], None)

        # Load the data back again
        ros_data = OdometryData.from_ros2_bag(bag_path, '/odom')

        # Make sure this data matches what we expect
        np.testing.assert_equal(float(ros_data.timestamps[13801]), 690.100000)
        np.testing.assert_array_equal(ros_data.positions[13801].astype(np.float128), [-66.153381, -76.155663, 1.445448])
        np.testing.assert_array_equal(ros_data.orientations[13801].astype(np.float128), [0.399908, 0.001246, -0.000566, 0.916554])
        np.testing.assert_equal(ros_data.frame_id, '/Husky1')
        np.testing.assert_equal(ros_data.child_frame_id, '/Husky1/base_link')


if __name__ == "__main__":
    unittest.main()