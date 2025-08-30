from robotdataprocess import ImageData, ImuData, OdometryData, CoordinateFrame
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    robot_name = "Drone2"

    # Extract image data from Hercules V1.4.1 to .npy
    rgb_data = ImageData.from_image_files('/media/dbutterfield3/T73/Hercules_datasets/V1.4.1/data/' + robot_name + '/rgb', 'front_center_Scene')
    rgb_data.to_npy('/media/dbutterfield3/T73/Hercules_datasets/V1.4.1/extract/files_for_roman_baseline/' + robot_name + '/rgb')


if __name__ == "__main__":
    main()