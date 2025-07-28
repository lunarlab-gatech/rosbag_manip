from robotdataprocess import ImageData, ImuData, OdometryData, CoordinateFrame
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    robot_name = "Drone2"

    # Extract depth data from Hercules v1.4 from individual .npy files to a single .npy file
    rgb_data = ImageData.from_npy_files('/media/dbutterfield3/T73/Hercules_datasets/V1.4/test2_2uav2ugv_calib_752x480/' + robot_name + '/depth', 'front_center_DepthPerspective')
    rgb_data.to_npy('/media/dbutterfield3/T73/Hercules_datasets/V1.4/extract/files_for_roman_baseline/' + robot_name + '/depth')


if __name__ == "__main__":
    main()