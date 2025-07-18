from robotdataprocess.data_types.OdometryData import OdometryData

def main():
    # Load csv files
    gt_drone_data = OdometryData.from_csv('/media/dbutterfield3/T71/Hercules_datasets/V1.2/extract/Drone1_odomGT.csv', 'Drone1/odom_local', 'Drone1/base_link')
    noisy_drone_data = OdometryData.from_csv('/media/dbutterfield3/T71/Hercules_datasets/V1.2/extract/Drone1_odomGTNoisy.csv',
                                       'Drone1/odom_local', 'Drone1/base_link')
    gt_Husky_data = OdometryData.from_csv('/media/dbutterfield3/T71/Hercules_datasets/V1.2/extract/Husky1_odomGT.csv', 'Husky1/odom_local', 'Husky1/base_link')
    noisy_Husky_data = OdometryData.from_csv('/media/dbutterfield3/T71/Hercules_datasets/V1.2/extract/Husky1_odomGTNoisy.csv',
                                       'Husky1/odom_local', 'Husky1/base_link')

    # Visualize it
    gt_drone_data.visualize([noisy_drone_data, gt_Husky_data, noisy_Husky_data], ['GT Odometry (Drone)', 'Noisy Odometry (Drone)',
                                                                                  'GT Odometry (Husky)', 'Noisy Odometry (Husky)'])

if __name__ == "__main__":
    main()