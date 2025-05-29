import cv2
import numpy as np
from pathlib import Path
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.store import Typestore
from collections import defaultdict
import tqdm

def resize_image_msg(msg: object, scale: float = 0.5) -> object:
    """
    Resize sensor_msgs/Image message. Currently supports 'rgb8' and '32FC1' encodings.
    
    Args:
        msg (object): The image message to resize.
        scale (float): The scale factor to resize the image. Default is 0.5 (50%).

    Returns:
        object: The resized image message.
    """

    # We retain original values, this isn't fully tested
    encoding = msg.encoding
    is_bigendian = msg.is_bigendian  

    # Resize based on encoding
    resized, resized_bytes, step = None, None, None
    if encoding == 'rgb8':
        img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        resized = cv2.resize(img_np, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        resized_bytes = np.frombuffer(resized.tobytes(), dtype=np.uint8)
        step = resized.shape[1] * 3 # channels

    elif encoding == '32FC1':
        img_np = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        resized = cv2.resize(img_np, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        resized_bytes = np.frombuffer(resized.tobytes(), dtype=np.uint8)
        step = resized.shape[1] * 4  # float32 = 4 bytes

    else:
        raise NotImplementedError(f"Unsupported encoding: {encoding}")

    # Update message fields
    msg.height = resized.shape[0]
    msg.width = resized.shape[1]
    msg.step = step
    msg.data = resized_bytes
    msg.encoding = encoding  # unchanged
    msg.is_bigendian = is_bigendian  # unchanged
    return msg

def downsample_bag(input_path: str, output_path: str, typestore: Typestore, topics_to_write: list,
                   downsample_rates: list):
    """
    Downsample a ROS2 bag file by downsampling the frequency of specified topics.
    Additionally resize image topics to half their original size.

    Args:
        input_path (str): Path to the input ROS2 bag folder.
        output_path (str): Path to the output ROS2 bag folder.
        typestore (Typestore): Typestore instance to use for message types.
        topics_to_write (list): List of topics to include in the downsampled bag.
        downsample_rates (list): List of downsample rates corresponding to each topic in topics_to_write.
    """

    # Convert to pathlib paths
    input_bag = Path(input_path)
    output_bag = Path(output_path)

    # Ensure we aren't overwriting an existing output bag
    if output_bag.exists():
        raise AssertionError("Delete Output Directory first!")
    
    # Calculate downsample ratios
    for val in downsample_rates:
        if val <= 0:
            raise ValueError("Downsample rates must be greater than 0.")
    downsample_ratios = dict(zip(topics_to_write, [1/x for x in downsample_rates]))

    # Open the bag
    with Reader(input_bag) as reader:
        connections = reader.connections
        
        # Create a writer with only the specified topics
        with Writer(output_bag, version=5) as writer:
            conn_map = {}
            for conn in connections:
                if conn.topic in topics_to_write:
                    conn_map[conn.topic] = writer.add_connection(
                        topic=conn.topic,
                        msgtype=conn.msgtype,
                        msgdef=conn.msgdef,
                        typestore=typestore,
                        serialization_format='cdr',
                        offered_qos_profiles=conn.ext.offered_qos_profiles
                    )

            # Initialize counters for each topic
            topic_counters = defaultdict(int)
            for topic in topics_to_write:
                topic_counters[topic] = 1

            # Setup progress bars
            pbarC = tqdm.tqdm(total=None, desc="Downsampling Requested Messages", unit=" messages")
            pbarW = tqdm.tqdm(total=None, desc="Writing Messages", unit=" messages")

            # Interate through messages in the bag
            connections = [x for x in reader.connections if x.topic in conn_map]
            for conn, timestamp, rawdata in reader.messages(connections=connections):

                # Check its a topic we want to write
                if conn.topic in conn_map:
                    pbarC.update(1)

                    # See if we need to save this message or cut it
                    if topic_counters[conn.topic] >= downsample_ratios[conn.topic]:
                        pbarW.update(1)

                        # Subtract integer portion of count
                        topic_counters[conn.topic] -= downsample_ratios[conn.topic]

                        # If it's an image topic, resize it
                        if conn.msgtype == 'sensor_msgs/msg/Image':
                            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                            resized_msg = resize_image_msg(msg)
                            serialized = typestore.serialize_cdr(resized_msg, conn.msgtype)
                            writer.write(conn_map[conn.topic], timestamp, serialized)
                        
                        # If its a camera_info topic, update it
                        elif conn.msgtype == 'sensor_msgs/msg/CameraInfo':
                            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                            msg.width = int(msg.width * 0.5)
                            msg.height = int(msg.height * 0.5)
                            serialized = typestore.serialize_cdr(msg, conn.msgtype)
                            writer.write(conn_map[conn.topic], timestamp, serialized)
                        
                        # Otherwise, just write the raw data
                        else:
                            writer.write(conn_map[conn.topic], timestamp, rawdata)

                    # Increment the counter for this topic
                    topic_counters[conn.topic] += 1

            # Close the progress bars
            pbarC.close()
            pbarW.close()
    
    print(f"Downsampling complete. Output bag saved to {output_bag}.")