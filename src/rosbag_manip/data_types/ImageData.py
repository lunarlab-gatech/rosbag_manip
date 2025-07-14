from __future__ import annotations

from ..conversion_utils import convert_collection_into_decimal_array
from .Data import Data
import decimal
from decimal import Decimal
from enum import Enum
import numpy as np
from numpy.lib.format import open_memmap
import os
from pathlib import Path
from PIL import Image
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.store import Typestore
from typeguard import typechecked
from typing import Tuple
import tqdm

class ImageData(Data):

    # Define image encodings enum
    class ImageEncoding(Enum):
        RGB8 = 0
        _32FC1 = 1
    
        @classmethod
        def from_str(cls, encoding_str: str):
            if encoding_str == "ImageEncoding.RGB8":
                return cls.RGB8
            elif encoding_str == "ImageEncoding._32FC1":
                return cls._32FC1
            else:
                raise NotImplementedError(f"This encoding ({encoding_str}) is not yet implemented (or it doesn't exist)!")
        
        @classmethod
        def from_ros_str(cls, encoding_str: str):
            encoding_str = encoding_str.lower()
            if encoding_str == 'rgb8':
                return ImageData.ImageEncoding.RGB8
            elif encoding_str == "32fc1":
                return ImageData.ImageEncoding._32FC1
            else:
                raise NotImplementedError(f"This encoding ({encoding_str}) is not yet implemented (or it doesn't exist)!")
        
        @classmethod
        def from_pillow_str(cls, encoding_str: str):
            if encoding_str == "RGB":
                return ImageData.ImageEncoding.RGB8
            else:
                raise NotImplementedError(f"This encoding ({encoding_str}) is not yet implemented (or it doesn't exist)!")
            
        @staticmethod
        def to_ros_str(encoding: ImageData.ImageEncoding):
            if encoding == ImageData.ImageEncoding.RGB8:
                return 'rgb8'
            if encoding == ImageData.ImageEncoding._32FC1:
                return '32FC1'
            else:
                raise NotImplementedError(f"This ImageData.ImageEncoding.{encoding} is not yet implemented (or it doesn't exist)!")


    # Define image-specific data attributes
    height: int
    width: int
    encoding: ImageEncoding
    images: np.ndarray

    @typechecked
    def __init__(self, frame_id: str, timestamps: np.ndarray | list, 
                 height: int, width: int, encoding: ImageData.ImageEncoding, images: np.ndarray):
        
        # Copy initial values into attributes
        super().__init__(frame_id, timestamps)
        self.height = height
        self.width = width
        self.encoding = encoding
        self.images = images

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
                encoding = ImageData.ImageEncoding.from_ros_str(msg.encoding)
                img = ImageData._decode_image_msg(msg, encoding, height, width)
                image_shape = img.shape
                break
        
        # Pre-allocate arrays (memory-mapped or otherwise)
        imgs_path = str(Path(save_folder) / "imgs.npy")
        os.makedirs(save_folder, exist_ok=True)
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
                img = None
                try:
                    img = ImageData._decode_image_msg(msg, encoding, height, width)
                except Exception as e:
                    print("Failure decoding image msg: ", e)
                if img is not None and img.shape == image_shape: 
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

        # Read in the attributes
        attr_data = {}
        with open(attr_path, "r") as f:
            for line in f:
                key, val = line.strip().split(":", 1)
                attr_data[key.strip()] = val.strip()

        # Parse and assign values to variables
        frame_id = attr_data["frame_id"]
        height = int(attr_data["height"])
        width = int(attr_data["width"])
        encoding = ImageData.ImageEncoding.from_str(attr_data["encoding"])

        # Create an ImageData class
        return cls(frame_id, np.load(ts_path), height, width, encoding, np.load(imgs_path, mmap_mode='r+'))
    
    @classmethod
    @typechecked
    def from_TartanAir(cls, folder_path: Path | str, img_folder_name: str, ts_folder_name: str,
                       frame_id: str):
        """
        Creates a class structure from the TartanAir dataset format, which includes
        images as .png files in a folder and timestamps as part of the pose file.

        Args:
            folder_path (Path | str): Path to the folder containing the TartanAir sequence.
            img_folder_name (str): Name of the folder containing the .png files.
            ts_folder_name (str): Name of the folder containing 'cam_time.npy'.
            frame_id (str): The frame where this image data was collected.
        Returns:
            ImageData: Instance of this class.
        """

        raise NotImplementedError("Sorting in this function is currently broken.")

        # Get all png files in the designated folder (sorted)
        img_folder_path = Path(folder_path) / img_folder_name
        all_image_files = [str(p) for p in sorted(Path(img_folder_path).glob("*.png"))]

        # Make sure the mode is what we expect
        first_image = Image.open(all_image_files[0])
        if first_image.mode != "RGB":
            raise NotImplementedError(f"Only RGB mode suppported for 'from_TartanAir', not \
                                      {first_image.mode}")

        # Load the images as numpy arrays
        images = np.zeros((len(all_image_files), first_image.height, first_image.width, 3), dtype=np.uint8)
        for i, path in enumerate(all_image_files):
            images[i] = np.array(Image.open(path), dtype=np.uint8)

        # Load the corresponding timestamps
        timestamps = convert_collection_into_decimal_array(np.load(Path(folder_path) / ts_folder_name / 'cam_time.npy'))
        
        # Create an ImageData class
        return cls(frame_id, timestamps, first_image.height, first_image.width, 
                   ImageData.ImageEncoding.from_pillow_str(first_image.mode), images)
    
    @classmethod
    @typechecked
    def from_image_files(cls, image_folder_path: Path | str, frame_id: str):
        """
        Creates a class structure from a folder with .png files, using the file names
        as the timestamps. This is the format that the HERCULES v1.3 dataset provides
        for image data.

        Args:
            image_folder_path (Path | str): Path to the folder with the images.
            frame_id (str): The frame where this image data was collected.
        Returns:
            ImageData: Instance of this class.
        """

        # Get all png files in the designated folder (sorted)
        all_image_files = [str(p) for p in Path(image_folder_path).glob("*.png")]

        # Extract the timestamps and sort them
        timestamps = convert_collection_into_decimal_array([s.split('/')[-1][:-4] for s in all_image_files])
        sorted_indices = np.argsort(timestamps)
        timestamps_sorted = timestamps[sorted_indices]

        # Use sorted_indices to sort all_image_files in the same way
        all_image_files_sorted = [all_image_files[i] for i in sorted_indices]

        # Make sure the mode is what we expect
        first_image = Image.open(all_image_files_sorted[0])
        if first_image.mode != "RGB":
            raise NotImplementedError(f"Only RGB mode suppported for 'from_image_files', not \
                                      {first_image.mode}")
        
        # Load the images as numpy arrays
        images = np.zeros((len(all_image_files_sorted), first_image.height, first_image.width, 3), dtype=np.uint8)
        pbar = tqdm.tqdm(total=len(all_image_files_sorted), desc="Extracting Images...", unit=" images")
        for i, path in enumerate(all_image_files_sorted):
            images[i] = np.array(Image.open(path), dtype=np.uint8)
            pbar.update()

        # Return an ImageData class
        return cls(frame_id, timestamps_sorted, first_image.height, first_image.width, 
                   ImageData.ImageEncoding.from_pillow_str(first_image.mode), images)

    # =========================================================================
    # ============================ Image Decoding ============================= 
    # ========================================================================= 

    
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

        # Find the locations in other where self timestamps would fit
        idxs = np.searchsorted(other.timestamps, self.timestamps, side='right')

        # Get the left indices and right indices
        idxs_right = np.clip(idxs, 0, len(other.timestamps)-1)
        idxs_left = np.clip(idxs - 1, 0, len(other.timestamps)-1)

        # Get distances to nearest on either side
        dists = np.minimum(np.abs(self.timestamps - other.timestamps[idxs_left]), 
                           np.abs(self.timestamps - other.timestamps[idxs_right]))
        
        # Print the mean and std of the distances
        print(f"Mean distance (left): {np.mean(np.abs(self.timestamps - other.timestamps[idxs_left]))}")
        print(f"Mean distance (right): {np.mean(np.abs(self.timestamps - other.timestamps[idxs_right]))}")
        print(f"Mean distance: {np.mean(dists)}")
        print(f"Std distance: {np.std(dists)}")

    # =========================================================================
    # =========================== Conversion to ROS =========================== 
    # ========================================================================= 

    @typechecked
    @staticmethod
    def get_ros_msg_type() -> str:
        """ Return the __msgtype__ for an Image msg. """
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        return typestore.types['sensor_msgs/msg/Image'].__msgtype__

    @typechecked
    def get_ros_msg(self, i: int):
        """
        Gets an Image ROS2 Humble message corresponding to the image represented by index i.
        
        Args:
            i (int): The index of the image message to convert.
        Raises:
            ValueError: If i is outside the data bounds.
        """

        # Check to make sure index is within data bounds
        if i < 0 or i >= self.len():
            raise ValueError(f"Index {i} is out of bounds!")

        # Get ROS2  message classes
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        Image, Header, Time = typestore.types['sensor_msgs/msg/Image'], typestore.types['std_msgs/msg/Header'], typestore.types['builtin_interfaces/msg/Time']

        # Calculate the step
        if self.encoding == ImageData.ImageEncoding.RGB8:
            step = 3 * self.width
        elif self.encoding == ImageData.ImageEncoding._32FC1:
            step = 4 * self.width

        # Get the seconds and nanoseconds
        seconds = int(self.timestamps[i])
        nanoseconds = (self.timestamps[i] - self.timestamps[i].to_integral_value(rounding=decimal.ROUND_DOWN)) * Decimal("1e9").to_integral_value(decimal.ROUND_HALF_EVEN)

        # Write the data into the new class
        return Image(Header(stamp=Time(sec=int(seconds), 
                                       nanosec=int(nanoseconds)), 
                            frame_id=self.frame_id),
                    height=self.height, 
                    width=self.width, 
                    encoding=ImageData.ImageEncoding.to_ros_str(self.encoding),
                    is_bigendian=0, 
                    step=step, 
                    data=self.images[i].flatten())
        