from robotdataprocess import ImageData, ImuData, OdometryData, CoordinateFrame
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    robot_name = "Drone2"

    # Extract depth data from Hercules V1.4.1 from individual .npy files to a single .npy file
    rgb_data = ImageData.from_npy_files('/media/dbutterfield3/T73/Hercules_datasets/V1.4.1/data/' + robot_name + '/depth', 'front_center_DepthPerspective')
    rgb_data.to_npy('/media/dbutterfield3/T73/Hercules_datasets/V1.4.1/extract/files_for_roman_baseline/' + robot_name + '/depth')


if __name__ == "__main__":
    main()