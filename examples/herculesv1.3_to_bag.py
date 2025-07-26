from robotdataprocess import ImageData, ImuData, OdometryData, CoordinateFrame
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    robot_name = "Drone1"

    # Extract RGB and IMU from Hercules v1.3
    imu_data = ImuData.from_txt_file('/media/dbutterfield3/Expansion/Hercules_Datasets/Archive/hercules_test_datasets_V1.3/test1_2uav2ugv_calib_752x480/' + robot_name + '/imu.txt', '' + robot_name + '/base_link', CoordinateFrame.NED)
    odom_data = OdometryData.from_txt_file('/media/dbutterfield3/Expansion/Hercules_Datasets/Archive/hercules_test_datasets_V1.3/test1_2uav2ugv_calib_752x480/' + robot_name + '/odom.txt', 'world', 'body', CoordinateFrame.NED)
    image_data = ImageData.from_image_files('/media/dbutterfield3/Expansion/Hercules_Datasets/Archive/hercules_test_datasets_V1.3/test1_2uav2ugv_calib_752x480/' + robot_name + '/rgb', '' + robot_name + '/front_center_Scene')

    # Convert data from NED frame to ROS frame (and make sure it is at the identity)
    odom_data.to_ROS_frame()
    odom_data.shift_to_start_at_identity()

    # Leave the IMU data in the NED frame (I believe that VINS-Mono actually adjusts internally)
    imu_data.frame = CoordinateFrame.FLU

    # Save it into a ROS2 Humble bag
    Ros2BagWrapper.write_data_to_rosbag('/media/dbutterfield3/T75/Hercules_datasets/V1.3/' + robot_name,
             [imu_data, image_data,  odom_data,       odom_data], 
             [  '/imu',    '/cam0', '/odom_gt', '/odom_gt/path'], 
             [    None,       None, "Odometry",          "Path"], 
             None)

if __name__ == "__main__":
    main()