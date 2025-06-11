import os
import numpy as np
from pathlib import Path
from rosbag_manip import rosbag_manipulation
from rosbags.rosbag1 import Reader as Reader1
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores, get_typestore
import unittest
import urllib.request

class TestRosbagManip(unittest.TestCase):
    """
    Test the rosbag_manip functionality.
    """

    @staticmethod
    def safe_urlretrieve(url, dest_path):
        """
        A method that retrives a file from a url, making sure
        to create the destination path if it doesn't exist.

        Throws:
            RuntimeError - If the recieved content is html.
        """
        os.makedirs(os.path.dirname(dest_path), exist_ok=True)
        with urllib.request.urlopen(url) as response:
            content_type = response.headers.get('Content-Type', '')
            if 'text/html' in content_type:
                raise RuntimeError(f"Failed to download {url}: received HTML instead of the expected file")
            with open(dest_path, 'wb') as out_file:
                out_file.write(response.read())

    def assert_two_msgs_match(self, ros1_bag: Path, ros2_bag: Path, topic: str):
        """
        Given a topic, manually check that the values in each message match.
        """
        
        # Setup Typestores
        stores = self.manipulator.create_typestores_with_external_msgs()
        typestore1 = stores[0]
        typestore2 = stores[1]

        # Load all messages of the specified type
        ros1_msgs, ros2_msgs = [], []
        with Reader1(ros1_bag) as reader1:
            connections1 = [x for x in reader1.connections if x.topic == topic]
            for conn1, timestamp1, rawdata1 in reader1.messages(connections=connections1):
                ros1_msgs.append(typestore1.deserialize_ros1(rawdata1, conn1.msgtype))

        with Reader2(ros2_bag) as reader2:
            connections2 = [x for x in reader2.connections if x.topic == topic]
            for conn2, timestamp2, rawdata2 in reader2.messages(connections=connections2):
                ros2_msgs.append(typestore2.deserialize_cdr(rawdata2, conn2.msgtype))

        # Check each of the attributes depending on topic
        for i in range(0, len(ros1_msgs)):
            msg1 = ros1_msgs[i]
            msg2 = ros2_msgs[i]
            if topic == '/hercules_node/Husky1/front_center_Scene/image':
                np.testing.assert_equal(msg1.header.frame_id, msg2.header.frame_id)
                np.testing.assert_equal(msg1.header.stamp.sec, msg2.header.stamp.sec)
                np.testing.assert_equal(msg1.header.stamp.nanosec, msg2.header.stamp.nanosec)
                np.testing.assert_equal(msg1.header.seq, 0)
                np.testing.assert_equal(msg1.height, msg2.height)
                np.testing.assert_equal(msg1.width, msg2.width)
                np.testing.assert_equal(msg1.encoding, msg2.encoding)
                np.testing.assert_equal(msg1.is_bigendian, msg2.is_bigendian)
                np.testing.assert_equal(msg1.step, msg2.step)
                np.testing.assert_array_equal(msg1.data, msg2.data)
            elif topic == '/hercules_node/Husky1/ground_truth/odom_local':
                np.testing.assert_equal(msg1.header.frame_id, msg2.header.frame_id)
                np.testing.assert_equal(msg1.header.stamp.sec, msg2.header.stamp.sec)
                np.testing.assert_equal(msg1.header.stamp.nanosec, msg2.header.stamp.nanosec)
                np.testing.assert_equal(msg1.header.seq, 0)
                np.testing.assert_equal(msg1.child_frame_id, msg2.child_frame_id)
                np.testing.assert_equal(msg1.pose.pose.position.x, msg2.pose.pose.position.x)
                np.testing.assert_equal(msg1.pose.pose.position.y, msg2.pose.pose.position.y)
                np.testing.assert_equal(msg1.pose.pose.position.z, msg2.pose.pose.position.z)
                np.testing.assert_equal(msg1.pose.pose.orientation.x, msg2.pose.pose.orientation.x)
                np.testing.assert_equal(msg1.pose.pose.orientation.y, msg2.pose.pose.orientation.y)
                np.testing.assert_equal(msg1.pose.pose.orientation.z, msg2.pose.pose.orientation.z)
                np.testing.assert_equal(msg1.pose.pose.orientation.w, msg2.pose.pose.orientation.w)
                np.testing.assert_array_equal(msg1.pose.covariance, msg2.pose.covariance)
                np.testing.assert_equal(msg1.twist.twist.linear.x, msg2.twist.twist.linear.x)
                np.testing.assert_equal(msg1.twist.twist.linear.y, msg2.twist.twist.linear.y)
                np.testing.assert_equal(msg1.twist.twist.linear.z, msg2.twist.twist.linear.z)
                np.testing.assert_equal(msg1.twist.twist.angular.x, msg2.twist.twist.angular.x)
                np.testing.assert_equal(msg1.twist.twist.angular.y, msg2.twist.twist.angular.y)
                np.testing.assert_equal(msg1.twist.twist.angular.z, msg2.twist.twist.angular.z)
                np.testing.assert_array_equal(msg1.twist.covariance, msg2.twist.covariance)
            elif topic == '/hercules_node/Drone2/front_center_DepthPlanar/camera_info':
                np.testing.assert_equal(msg1.header.frame_id, msg2.header.frame_id)
                np.testing.assert_equal(msg1.header.stamp.sec, msg2.header.stamp.sec)
                np.testing.assert_equal(msg1.header.stamp.nanosec, msg2.header.stamp.nanosec)
                np.testing.assert_equal(msg1.header.seq, 0)
                np.testing.assert_equal(msg1.height, msg2.height)
                np.testing.assert_equal(msg1.width, msg2.width)
                np.testing.assert_equal(msg1.distortion_model, msg2.distortion_model)
                np.testing.assert_array_equal(msg1.D, msg2.d)
                np.testing.assert_array_equal(msg1.K, msg2.k)
                np.testing.assert_array_equal(msg1.R, msg2.r)
                np.testing.assert_array_equal(msg1.P, msg2.p)
                np.testing.assert_equal(msg1.binning_x, msg2.binning_x)
                np.testing.assert_equal(msg1.binning_y, msg2.binning_y)
                np.testing.assert_equal(msg1.roi.x_offset, msg2.roi.x_offset)
                np.testing.assert_equal(msg1.roi.y_offset, msg2.roi.y_offset)
                np.testing.assert_equal(msg1.roi.height, msg2.roi.height)
                np.testing.assert_equal(msg1.roi.width, msg2.roi.width)
                np.testing.assert_equal(msg1.roi.do_rectify, msg2.roi.do_rectify)
            elif topic == '/hercules_node/Drone2/imu/imu':
                np.testing.assert_equal(msg1.header.frame_id, msg2.header.frame_id)
                np.testing.assert_equal(msg1.header.stamp.sec, msg2.header.stamp.sec)
                np.testing.assert_equal(msg1.header.stamp.nanosec, msg2.header.stamp.nanosec)
                np.testing.assert_equal(msg1.header.seq, 0)
                np.testing.assert_equal(msg1.orientation.x, msg2.orientation.x)
                np.testing.assert_equal(msg1.orientation.y, msg2.orientation.y)
                np.testing.assert_equal(msg1.orientation.z, msg2.orientation.z)
                np.testing.assert_equal(msg1.orientation.w, msg2.orientation.w)
                np.testing.assert_array_equal(msg1.orientation_covariance, msg2.orientation_covariance)
                np.testing.assert_equal(msg1.angular_velocity.x, msg2.angular_velocity.x)
                np.testing.assert_equal(msg1.angular_velocity.y, msg2.angular_velocity.y)
                np.testing.assert_equal(msg1.angular_velocity.z, msg2.angular_velocity.z)
                np.testing.assert_array_equal(msg1.angular_velocity_covariance, msg2.angular_velocity_covariance)
                np.testing.assert_equal(msg1.linear_acceleration.x, msg2.linear_acceleration.x)
                np.testing.assert_equal(msg1.linear_acceleration.y, msg2.linear_acceleration.y)
                np.testing.assert_equal(msg1.linear_acceleration.z, msg2.linear_acceleration.z)
                np.testing.assert_array_equal(msg1.linear_acceleration_covariance, msg2.linear_acceleration_covariance)
            elif topic == '/tf' or topic == '/tf_static':
                np.testing.assert_equal(len(msg1.transforms), len(msg2.transforms))
                for i in range(0, len(msg1.transforms)):
                    trans1 = msg1.transforms[i]
                    trans2 = msg2.transforms[i]
                    np.testing.assert_equal(trans1.header.frame_id, trans2.header.frame_id)
                    np.testing.assert_equal(trans1.header.stamp.sec, trans2.header.stamp.sec)
                    np.testing.assert_equal(trans1.header.stamp.nanosec, trans2.header.stamp.nanosec)
                    np.testing.assert_equal(trans1.header.seq, 0)
                    np.testing.assert_equal(trans1.child_frame_id, trans2.child_frame_id)
                    np.testing.assert_equal(trans1.transform.translation.x, trans2.transform.translation.x)
                    np.testing.assert_equal(trans1.transform.translation.y, trans2.transform.translation.y)
                    np.testing.assert_equal(trans1.transform.translation.z, trans2.transform.translation.z)
                    np.testing.assert_equal(trans1.transform.rotation.x, trans2.transform.rotation.x)
                    np.testing.assert_equal(trans1.transform.rotation.y, trans2.transform.rotation.y)
                    np.testing.assert_equal(trans1.transform.rotation.z, trans2.transform.rotation.z)
                    np.testing.assert_equal(trans1.transform.rotation.w, trans2.transform.rotation.w)
            else:
                raise NotImplementedError(f"Tests are not implemented for this topic: {topic}")

    def test_convert_ros2_to_ros1(self):
        """
        Test that all message types supported have their values
        correctly carried over from the ros2 bag into the ros1
        bag, and that no messages are lost.
        """

        # Get path to test bags & external messages
        path_hercules_bag = Path(Path('.'), 'tests', 'test_bags', 'hercules_test_bag_pruned_3').absolute()
        path_hercules_bag_ros1 = path_hercules_bag.parent / 'hercules_test_bag_pruned_3.bag'
        path_external_msgs_ros2 = Path(Path('.').parent, 'external_msgs_ros2').absolute()
        path_external_msgs_ros1 = Path(Path('.').parent, 'external_msgs_ros1').absolute()

        # Get paths to files within the input bag
        path_hercules_bag_db3 = path_hercules_bag / Path("hercules_test_bag_pruned_3.db3")
        path_hercules_bag_yaml = path_hercules_bag / Path("metadata.yaml")

        # Download the test bag (as its too big for GitHub)
        if not os.path.isfile(path_hercules_bag_db3):
            TestRosbagManip.safe_urlretrieve("https://www.dropbox.com/scl/fi/2flzok4kwb42no4vqr0ie/hercules_test_bag_pruned_3.db3?rlkey=8yj3sbo3vp96513qvxh26q0di&st=kxhx3hep&dl=1", path_hercules_bag_db3)
        if not os.path.isfile(path_hercules_bag_yaml):
            TestRosbagManip.safe_urlretrieve("https://www.dropbox.com/scl/fi/94jshmjb5l3lcb71g64yq/metadata.yaml?rlkey=we4ex9bpd81shjtxkvm1gfl26&st=9o4evtni&dl=1", path_hercules_bag_yaml)

        # Setup a dictionary with configuration parameters 
        config_dict = {
            "input_bag": path_hercules_bag,
            "output_bag": path_hercules_bag_ros1,
            "external_msgs_path_ros2": path_external_msgs_ros2,
            "external_msgs_path_ros1": path_external_msgs_ros1,
            "operation_to_run": 'convert_ros2_to_ros1'
        }

        # If a ROS1 bag exists from a previous test, delete it
        if os.path.isfile(path_hercules_bag_ros1):
            os.remove(path_hercules_bag_ros1)

        # Call the operation to write ROS1 bag
        self.manipulator = rosbag_manipulation(**config_dict)

        # Read the ROS2 bag and count number of messages on each topic
        topic_counts = {}
        with Reader2(path_hercules_bag) as reader:
            for conn, timestamp, rawdata in reader.messages():
                topic = conn.topic
                try:
                    topic_counts[topic] += 1
                except:
                    topic_counts[topic] = 1

        # Read the ROS1 bag and count the number of messages as well
        topic_counts_ros1 = {}
        with Reader1(path_hercules_bag_ros1) as reader:
            for conn, timestamp, rawdata in reader.messages():
                topic = conn.topic
                try:
                    topic_counts_ros1[topic] += 1
                except:
                    topic_counts_ros1[topic] = 1

        # Check that both bags have the same number of each topic
        ros2_topics = sorted(list(topic_counts.keys()))
        ros1_topics = sorted(list(topic_counts_ros1.keys()))
        np.testing.assert_array_equal(ros1_topics, ros2_topics)
        for key in ros2_topics:
            np.testing.assert_equal(topic_counts_ros1[key], topic_counts[key])

        # For certain topics, check that each message in each bag match exactly
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/hercules_node/Husky1/front_center_Scene/image')
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/hercules_node/Husky1/ground_truth/odom_local')
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/hercules_node/Drone2/front_center_DepthPlanar/camera_info')
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/hercules_node/Drone2/imu/imu')
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/tf_static')
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/tf')

if __name__ == "__main__":
    unittest.main()