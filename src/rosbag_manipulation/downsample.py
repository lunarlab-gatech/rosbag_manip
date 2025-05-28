import cv2
import numpy as np
from pathlib import Path
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore
from collections import defaultdict
import tqdm

# Config
CAMERA_HZ = 35
TF_HZ = 400
CAMERA_DOWNSAMPLE_RATIO = 1 #int(CAMERA_HZ / 3.5)
TF_DOWNSAMPLE_RATIO = 1 #int(TF_HZ / 40)
START_TS = 1747048365868941009 # nanoseconds
MAX_DURATION = 60 # seconds

CAMERA_TOPIC_SUFFIXES = [
    'front_center_Scene',
    'front_center_DepthPerspective',
]

MSG_TYPE_SUFFIXES = [
    'image',
    'camera_info',
]

ROBOT_NAMES = ['Husky1', 'Husky2', 'Drone1', 'Drone2']  # Replace with your actual robot names

def resize_image_msg(msg, scale=0.5):
    """Resize sensor_msgs/Image message, supporting 'rgb8' and '32FC1' encodings."""
    encoding = msg.encoding
    is_bigendian = msg.is_bigendian  # Usually 0, but we retain original value

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

def downsample_bag(input_bag_path, output_bag_path):
    input_bag = Path(input_bag_path)
    output_bag = Path(output_bag_path)

    if output_bag.exists():
        raise AssertionError("Delete Output Directory first!")

    topics_to_write = []
    for robot_name in ROBOT_NAMES:
        for suffix in CAMERA_TOPIC_SUFFIXES:
            for msg_type in MSG_TYPE_SUFFIXES:
                topics_to_write.append(f'/hercules_node/{robot_name}/{suffix}/{msg_type}')
    topics_to_write.append('/tf')
    topics_to_write.append('/tf_static')

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(input_bag) as reader:
        connections = reader.connections

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

            image_counters = defaultdict(int)
            info_counters = defaultdict(int)
            tf_counter = 0
            static_tf_counter = 0

            # setup tqdm 
            pbar = tqdm.tqdm(total=4124, desc="Writing Images", unit="frames")

            for conn, timestamp, rawdata in reader.messages():
                topic = conn.topic

                # Only include first 60 seconds
                if ((timestamp - START_TS) / 1e9) > 60:
                    if topic == '/hercules_node/Husky1/front_center_Scene/image':
                        pbar.update()
                    continue

                if topic in conn_map and 'image' in topic:
                    count = image_counters[topic]
                    if count % CAMERA_DOWNSAMPLE_RATIO == 0:
                        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                        resized_msg = resize_image_msg(msg)
                        serialized = typestore.serialize_cdr(resized_msg, conn.msgtype)
                        writer.write(conn_map[topic], timestamp, serialized)
                    image_counters[topic] += 1
                    if topic == '/hercules_node/Husky1/front_center_Scene/image':
                        pbar.update()

                elif topic in conn_map and 'camera_info' in topic:
                    count = info_counters[topic]
                    if count % CAMERA_DOWNSAMPLE_RATIO == 0:
                        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                        msg.width = int(msg.width * 0.5)
                        msg.height = int(msg.height * 0.5)
                        serialized = typestore.serialize_cdr(msg, conn.msgtype)
                        writer.write(conn_map[topic], timestamp, serialized)
                    info_counters[topic] += 1

                elif topic == '/tf':
                    if tf_counter % TF_DOWNSAMPLE_RATIO == 0:
                        writer.write(conn_map[topic], timestamp, rawdata)
                    tf_counter += 1

                elif topic == '/tf_static':
                    if static_tf_counter % TF_DOWNSAMPLE_RATIO == 0:
                        writer.write(conn_map[topic], timestamp, rawdata)
                    static_tf_counter += 1