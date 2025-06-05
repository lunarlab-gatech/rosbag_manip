import os
import numpy as np
from pathlib import Path
from rosbag_manip import rosbag_manipulation
from rosbags.rosbag1 import Reader as Reader1
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores, get_typestore
import unittest

class TestRosbagManip(unittest.TestCase):
    """
    Test the rosbag_manip functionality.
    """

    def assert_two_msgs_match(self, ros1_bag: Path, ros2_bag: Path, topic: str):
        """
        Given a topic, manually check that the values in each message match.
        """
        
        # Setup Typestores
        typestore1 = get_typestore(Stores.ROS1_NOETIC)
        typestore2 = get_typestore(Stores.ROS2_HUMBLE)

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
            elif topic == '/hercules_node/Husky1/odom_local':
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
            elif topic == '/hercules_node/Drone2/front_center_DepthPerspective/camera_info':
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
        path_external_msgs = Path(Path('.').parent, 'external_msgs').absolute()

        # Setup a dictionary with configuration parameters 
        config_dict = {
            "input_bag": path_hercules_bag,
            "output_bag": path_hercules_bag_ros1,
            "external_msgs_path_ros2": path_external_msgs,
            "operation_to_run": 'convert_ros2_to_ros1'
        }

        # If a ROS1 bag exists from a previous test, delete it
        if os.path.isfile(path_hercules_bag_ros1):
            os.remove(path_hercules_bag_ros1)

        # Call the operation to write ROS1 bag
        manipulator = rosbag_manipulation(**config_dict)

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
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/hercules_node/Husky1/odom_local')
        self.assert_two_msgs_match(path_hercules_bag_ros1, path_hercules_bag, '/hercules_node/Drone2/front_center_DepthPerspective/camera_info')

if __name__ == "__main__":
    unittest.main()