from robotdataprocess import ImageData, ImuData, OdometryData, CoordinateFrame
from robotdataprocess.rosbag.Ros2BagWrapper import Ros2BagWrapper

def main():
    robot_name = "Drone2"

    # Load the odometry data and reformat the csv in a ROMAN friendly format
    odom_data = OdometryData.from_csv('/home/dbutterfield3/Research/ros_workspaces/vins_mono_ws/src/VINS-MONO-ROS2/output/hercules/'+robot_name+'/vins_result_no_loop.csv', "odom", 'base_link', CoordinateFrame.FLU, False, None)
    odom_data.to_csv('/media/dbutterfield3/T73/Hercules_datasets/V1.4/extract/files_for_roman_baseline/'+robot_name+'/vins_result_no_loop_reformatted.csv')

if __name__ == "__main__":
    main()