from rosbag_manip import ImageData, ImuData, OdometryData
from rosbag_manip.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    # Extract RGB and IMU from Hercules v1.3
    imu_data = ImuData.from_txt_file('/media/dbutterfield3/Expansion/Hercules_Datasets/Archive/hercules_test_datasets_V1.3/test1_2uav2ugv_calib_752x480/Husky1/imu.txt', 'Husky1/base_link')
    image_data = ImageData.from_image_files('/media/dbutterfield3/Expansion/Hercules_Datasets/Archive/hercules_test_datasets_V1.3/test1_2uav2ugv_calib_752x480/Husky1/rgb', 'Husky1/front_center_Scene')
    odom_data = OdometryData.from_txt_file('/media/dbutterfield3/Expansion/Hercules_Datasets/Archive/hercules_test_datasets_V1.3/test1_2uav2ugv_calib_752x480/Husky1/odom.txt', 'Husky1', 'Husky1/base_link')

    # Save it into a ROS2 Humble bag
    Ros2BagWrapper.write_data_to_rosbag('/media/dbutterfield3/T72/Hercules_datasets/V1.3/test1_2uav2ugv_calib_752x480', [imu_data, image_data, odom_data], ['/imu', '/cam0', '/odom_gt'], None)

if __name__ == "__main__":
    main()