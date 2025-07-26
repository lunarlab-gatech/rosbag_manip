from robotdataprocess.data_types.OdometryData import OdometryData
from robotdataprocess import CoordinateFrame

def main():
    # Load csv files
    gt_drone_data = OdometryData.from_csv('/media/dbutterfield3/T75/Hercules_datasets/V1.2/extract/Husky2_odomGT.csv', 'Husky2/odom_local', 'Husky2/base_link', CoordinateFrame.FLU)
    noisy_drone_data = OdometryData.from_csv('/media/dbutterfield3/T75/Hercules_datasets/V1.2/extract/Husky2_odomGTNoisy.csv',
                                       'Husky2/odom_local', 'Husky2/base_link', CoordinateFrame.FLU)
    gt_Husky_data = OdometryData.from_csv('/media/dbutterfield3/T75/Hercules_datasets/V1.2/extract/Husky1_odomGT.csv', 'Husky1/odom_local', 'Husky1/base_link', CoordinateFrame.FLU)
    noisy_Husky_data = OdometryData.from_csv('/media/dbutterfield3/T75/Hercules_datasets/V1.2/extract/Husky1_odomGTNoisy.csv',
                                       'Husky1/odom_local', 'Husky1/base_link', CoordinateFrame.FLU)

    # Visualize it
    gt_drone_data.visualize([noisy_drone_data, gt_Husky_data, noisy_Husky_data], 
                            ['GT Odometry (Husky2)', 'Noisy Odometry (Husky2)',
                             'GT Odometry (Husky)', 'Noisy Odometry (Husky)'])

if __name__ == "__main__":
    main()