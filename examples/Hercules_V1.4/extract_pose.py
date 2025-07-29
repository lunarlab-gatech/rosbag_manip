from robotdataprocess import ImageData, ImuData, OdometryData, CoordinateFrame
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    robot_name = "Drone2"

    # Load the odometry data
    odom_data = OdometryData.from_txt_file('/media/dbutterfield3/T73/Hercules_datasets/V1.4/test2_2uav2ugv_calib_752x480/' + robot_name + '/odom.txt', robot_name + '/odom', robot_name + '/ground_truth/base_link', CoordinateFrame.NED)

    # Convert to the FLU coordinate frame
    odom_data.to_FLU_frame()

    # Save back to a csv file
    odom_data.to_csv('/media/dbutterfield3/T73/Hercules_datasets/V1.4/extract/files_for_roman_baseline/' + robot_name + '/poseGT.csv')

if __name__ == "__main__":
    main()