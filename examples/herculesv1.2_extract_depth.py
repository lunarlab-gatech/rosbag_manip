from robotdataprocess import ImageData, ImuData, OdometryData, CoordinateFrame
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    # Extract Depth data from Hercules v1.2
    image_data = ImageData.from_ros2_bag('/media/dbutterfield3/T75/Hercules_datasets/V1.2/ausenv_test1_checkpoints_CSLAM_2UAVUGV', '/hercules_node/Husky2/front_center_DepthPlanar/image', '/media/dbutterfield3/T75/Hercules_datasets/V1.2/extract/Husky2_Depth')

if __name__ == "__main__":
    main()