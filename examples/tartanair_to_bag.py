from rosbag_manip.data_types.ImageData import ImageData
from rosbag_manip.data_types.ImuData import ImuData
from rosbag_manip.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    # Extract data from TartanGround Dataset
    imu_data = ImuData.from_TartanAir('/home/dbutterfield3/Research/TartanAir/dataset/TartanGround/OldTownSummer/Data_ground/Pose2000/imu', 'imu')
    image_data = ImageData.from_TartanAir('/home/dbutterfield3/Research/TartanAir/dataset/TartanGround/OldTownSummer/Data_ground/Pose2000', 'image_lcam_front', 'imu', 'image_lcam_front')

    # Save it into a ROS2 Humble bag
    Ros2BagWrapper.write_data_to_rosbag('/home/dbutterfield3/Research/rosbag_manip/outputs/tartanGround', [imu_data, image_data], ['/imu', '/image_lcam_front'], None)

if __name__ == "__main__":
    main()