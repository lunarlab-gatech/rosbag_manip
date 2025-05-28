from pathlib import Path
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.store import Typestore
import tqdm


def crop_bag(input_path: str, output_path: str, typestore: Typestore, start_ts: float, end_ts: float):
    """
    Crop a ROS2 bag file to only include messages within the specified time range. Note
    that this function uses the timestamps of when the messages were written to the bag,
    not the timestamps in the headers of the messages themselves.

    Args:
        input_path (str): Path to the input ROS2 bag folder.
        output_path (str): Path to the output ROS2 bag folder.
        typestore (Typestore): Typestore instance to use for message types.
        start_ts (float): Start timestamp in seconds for cropping the bag.
        end_ts (float): End timestamp in seconds for cropping the bag. If None, no end timestamp is set.
    """

    # Convert to pathlib paths
    input_bag = Path(input_path)
    output_bag = Path(output_path)

    # Make sure we aren't overwriting an existing output bag
    if output_bag.exists():
        raise AssertionError("Delete Output Directory first!")

    with Reader(input_bag) as reader:
        connections = reader.connections
        
        # Setup the bag writer
        with Writer(output_bag, version=5) as writer:
            conn_map = {}
            for conn in connections:
                conn_map[conn.topic] = writer.add_connection(
                    topic=conn.topic,
                    msgtype=conn.msgtype,
                    msgdef=conn.msgdef,
                    typestore=typestore,
                    serialization_format='cdr',
                    offered_qos_profiles=conn.ext.offered_qos_profiles
                )

            # setup tqdm 
            pbarW = tqdm.tqdm(total=None, desc="Writing messages", unit=" messages")

            # Iternate through messages in the bag
            for conn, timestamp, rawdata in reader.messages(start=start_ts * 1e9, stop=end_ts * 1e9):
                writer.write(conn_map[conn.topic], timestamp, rawdata)
                pbarW.update(1)