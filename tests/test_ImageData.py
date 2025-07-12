from decimal import Decimal
import numpy as np
import os
from pathlib import Path
from rosbag_manip.data_types.ImageData import ImageData
from test_utils import safe_urlretrieve
import unittest


class TestImageData(unittest.TestCase):
    
    def setUp(self):
        """ 
        Setup paths and download files that will be used 
        for all tests.
        """

        # Get paths to test bags & external messages
        self.path_hercules_bag = Path(Path('.'), 'tests', 'test_bags', 'hercules_test_bag_pruned_3').absolute()
        self.path_external_msgs_ros2 = Path(Path('.').parent, 'external_msgs_ros2').absolute()
        self.path_external_msgs_ros1 = Path(Path('.').parent, 'external_msgs_ros1').absolute()

        # Get paths to files within the input bag
        path_hercules_bag_db3 = self.path_hercules_bag / Path("hercules_test_bag_pruned_3.db3")
        path_hercules_bag_yaml = self.path_hercules_bag / Path("metadata.yaml")

        # Download the test bag (as its too big for GitHub)
        if not os.path.isfile(path_hercules_bag_db3):
            safe_urlretrieve("https://www.dropbox.com/scl/fi/r3qxkbypaiq3o277qu9ad/hercules_test_bag_pruned_3.db3?rlkey=uumrmpt80elj2gjhqls6027pm&st=4g498h35&dl=1", path_hercules_bag_db3)
        if not os.path.isfile(path_hercules_bag_yaml):
            safe_urlretrieve("https://www.dropbox.com/scl/fi/alze2h2e3h4l09f55uka9/metadata.yaml?rlkey=may9dvginz3bg6gsgtgcod3m7&st=ypw42mhh&dl=1", path_hercules_bag_yaml)

    def test_from_ros_str(self):
        """ Make sure that an exception is thrown with a non-valid ROS encoding str"""
        with np.testing.assert_raises(NotImplementedError):
            ImageData.ImageEncoding.from_ros_str("fake_name")

    def test_from_npy(self):
        """
        Test that create of ImgData from an .npy file results 
        in the same class as loading from the rosbag.
        """

        # Convert the data into .npy (byproduct of loading ImageData from rosbag)
        save_folder = Path(Path('.'), 'tests', 'test_outputs', 'test_from_npy').absolute()
        rosData = ImageData.from_ros2_bag(self.path_hercules_bag, '/hercules_node/Drone2/front_center_Scene/image', save_folder)

        # Load the .npy file
        npyData = ImageData.from_npy(save_folder)

        # Make sure the two classes are equivalent
        np.testing.assert_equal(rosData.frame_id, npyData.frame_id)
        np.testing.assert_array_equal(rosData.timestamps, npyData.timestamps)
        np.testing.assert_equal(rosData.height, npyData.height)
        np.testing.assert_equal(rosData.width, npyData.width)
        np.testing.assert_equal(rosData.encoding, npyData.encoding)
        np.testing.assert_array_equal(rosData.images, npyData.images)
    
        # Make sure that there is actually data in the test bag that we are preserving
        if rosData.height == 0 or rosData.width == 0 or len(rosData.images) == 0 \
            or len(rosData.timestamps) == 0 or rosData.frame_id == "":
            self.fail("Test data is not representative of real-time operation!")

if __name__ == "__main__":
    unittest.main()