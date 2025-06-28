from __future__ import annotations

from ..conversion_utils import convert_collection_into_decimal_array
from decimal import Decimal
from enum import Enum
import numpy as np
from numpy.lib.format import open_memmap
from pathlib import Path
from ..rosbag.Ros2BagWrapper import Ros2BagWrapper
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys.store import Typestore
from typeguard import typechecked
from typing import Tuple
import tqdm

class ImageData:

    # Define image encodings enum
    class ImageEncoding(Enum):
        RGB8 = 0
        _32FC1 = 1

    # Define data attributes
    frame_id: str
    timestamps: np.ndarray[Decimal]
    height: int
    width: int
    encoding: ImageEncoding
    images: np.ndarray

    @typechecked
    def __init__(self, frame_id: str, timestamps: np.ndarray | list, 
                 height: int, width: int, encoding: ImageData.ImageEncoding, images: np.ndarray):
        
        # Copy initial values into attributes
        self.frame_id = frame_id
        self.timestamps = convert_collection_into_decimal_array(timestamps)
        self.height = height
        self.width = width
        self.encoding = encoding
        self.images = images

        # Check to ensure that all timestamps are sequential
        for i in range(len(self.timestamps) - 1):
            if self.timestamps[i] >= self.timestamps[i+1]:
                raise ValueError("Timestamps do not come in sequential order!")

    # =========================================================================
    # ============================ Class Methods ============================== 
    # =========================================================================  

    @classmethod
    @typechecked
    def from_ros2_bag(cls, bag_path: Path | str, img_topic: str, save_folder: Path | str):
        """
        Creates a class structure from a ROS2 bag file with an Image topic. Will
        Also save all the data into .npy and .txt files as this is required if image
        data doesn't fit into the RAM.

        Args:
            bag_path (Path | str): Path to the ROS2 bag file.
            img_topic (str): Topic of the Image messages.
            save_folder (Path | str): Path to save class data into.
        Returns:
            ImageData: Instance of this class.
        """

        # Get topic message count and typestore
        bag_wrapper = Ros2BagWrapper(bag_path, None)
        typestore: Typestore = bag_wrapper.get_typestore()
        num_msgs: int = bag_wrapper.get_topic_count(img_topic)

        # Extract relevant image parameters
        image_shape, frame_id, height, width, encoding = None, None, None, None, None
        with Reader2(bag_path) as reader:
            connections = [x for x in reader.connections if x.topic == img_topic]
            for conn, _, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                frame_id = msg.header.frame_id
                height = msg.height
                width = msg.width
                encoding = ImageData._convert_ros_encoding_to_enum(msg.encoding.lower())
                img = ImageData._decode_image_msg(msg, encoding, height, width)
                image_shape = img.shape
                break
        
        # Pre-allocate arrays (memory-mapped or otherwise)
        imgs_path = str(Path(save_folder) / "imgs.npy")
        img_memmap = open_memmap(imgs_path, dtype=img.dtype, shape=(num_msgs, *image_shape), mode='w+')
        timestamps_np = np.zeros(num_msgs, dtype=np.float128)

        # Setup tqdm bar
        pbar = tqdm.tqdm(total=num_msgs, desc="Extracting Images...", unit=" msgs")

        # Extract the images/timestamps and save
        with Reader2(bag_path) as reader: 
            i = 0
            connections = [x for x in reader.connections if x.topic == img_topic]
            for conn, _, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

                # Extract images (skipping malformed ones)
                img = ImageData._decode_image_msg(msg, encoding, height, width)
                if img.shape == image_shape: 
                    img_memmap[i] = img

                # Extract timestamps
                ts = Ros2BagWrapper.extract_timestamp(msg)
                timestamps_np[i] = ts

                # Update the count
                i += 1
                pbar.update(1)

        # Write all images to disk and save timestamps and other data
        img_memmap.flush()
        np.save(str(Path(save_folder) / "times.npy"), timestamps_np, allow_pickle=False)
        with open(str(Path(save_folder) / "attributes.txt"), "w") as f:
            f.write(f"image_shape: {image_shape}\n")
            f.write(f"frame_id: {frame_id}\n")
            f.write(f"height: {height}\n")
            f.write(f"width: {width}\n")
            f.write(f"encoding: {encoding}\n")

        # Create an ImageData class
        return cls(frame_id, timestamps_np, height, width, encoding, np.load(imgs_path, mmap_mode='r+'))
    
    @classmethod
    @typechecked
    def from_npy(cls, folder_path: Path | str):
        """
        Creates a class structure from .npy and .txt files (the ones written by from_ros2_bag()).

        Args:
            folder_path (Path | str): Path to the folder with:
                - imgs.npy
                - times.npy
                - attributes.txt
        Returns:
            ImageData: Instance of this class.
        """

        # Calculate other paths from folder path
        imgs_path = str(Path(folder_path) / "imgs.npy")
        ts_path = str(Path(folder_path) / "times.npy")
        attr_path = str(Path(folder_path) / "attributes.txt")

        # Read in the attirubtes
        attr_data = {}
        with open(attr_path, "r") as f:
            for line in f:
                key, val = line.strip().split(":", 1)
                attr_data[key.strip()] = val.strip()

        # Parse and assign values to variables
        frame_id = attr_data["frame_id"]
        height = int(attr_data["height"])
        width = int(attr_data["width"])
        encoding = ImageData.ImageEncoding[attr_data["encoding"]]  # Convert string to enum

        # Create an ImageData class
        return cls(frame_id, np.load(ts_path), height, width, encoding, np.load(imgs_path, mmap_mode='r+'))

    # =========================================================================
    # ============================ Image Decoding ============================= 
    # ========================================================================= 

    @staticmethod
    def _convert_ros_encoding_to_enum(encoding_str: str) -> ImageData.ImageEncoding:
        """
        Convert ROS image encoding str to ImageEncoding enum.

        Args:
            encoding_str (str): ROS2 image encoding str to convert.
        Returns:
            ImageEncoding: Enum compatible with this class.
        Raises:
            NotImplementedError: If encoding_str doesn't have an implemented
                ImageEncoding yet.
        """

        if encoding_str == 'rgb8':
            return ImageData.ImageEncoding.RGB8
        elif encoding_str == "32fc1":
            return ImageData.ImageEncoding._32FC1
        else:
            raise NotImplementedError(f"This encoding ({encoding_str}) is not yet implemented (or it doesn't exist)!")
    
    @staticmethod
    def _map_encoding_to_dtype_and_channels(encoding: ImageData.ImageEncoding) -> Tuple[type, int]:
        """ 
        Helper method that maps an image encoding to the corresponding dtype and channels.
        
        Raises:
            NotImplementedError: If this mapping hasn't been defined
        """
        match encoding:
            case ImageData.ImageEncoding.RGB8:
                return (np.uint8, 3)
            case ImageData.ImageEncoding._32FC1:
                return (np.float32, 1)
            case _:
                raise NotImplementedError(f"This encoding ({encoding}) is missing a mapping to dtype/channels!")
            

    @staticmethod
    @typechecked
    def _decode_image_msg(msg: object, encoding: ImageData.ImageEncoding, height: int, width: int):
        """
        Helper method that decodes image data from a ROS2 Image message.

        Args:
            msg (object): The ROS2 Image message.
            encoding (ImageEncoding): The encoding of the image data.
            height (int): Height of the image.
            width (int): Width of the image .
        """
        dtype, channels = ImageData._map_encoding_to_dtype_and_channels(encoding)
        if channels > 1:
            return np.frombuffer(msg.data, dtype=dtype).reshape((height, width, channels)) 
        else:
            return np.frombuffer(msg.data, dtype=dtype).reshape((height, width))
        
    # =========================================================================
    # ========================= Timestamp comparison ========================== 
    # ========================================================================= 

    def compare_timestamps(self, other: ImageData):
        """
        This method compares two ImageData objects based on the timestamps of their
        images.
        """

        # For each timestamp in self, find the distance to the closest timestamp in other