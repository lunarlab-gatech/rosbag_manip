from robotdataprocess.data_types.OdometryData import OdometryData
from robotdataprocess import CoordinateFrame

def main():
    robot_names = ["Husky1", "Husky2"]

    # Load csv files
    data: list[OdometryData] = []
    for name in robot_names:
        d = OdometryData.from_csv('/media/dbutterfield3/T73/Hercules_datasets/V1.4/extract/files_for_roman_baseline/'+name+'/odomGT.csv', "odom", 'ground_truth/base_link', CoordinateFrame.FLU, True, None)
        data.append(d)

    # Load VINS estimated results
    for name in robot_names:
        print(name)
        d = OdometryData.from_csv('/media/dbutterfield3/T73/Hercules_datasets/V1.4/extract/files_for_roman_baseline/'+name+'/vins_result_no_loop.csv', "odom", 'base_link', CoordinateFrame.FLU, False, None)
        data.append(d)

    # Visualize it
    robot_names_alt = [x + "(VINS-Mono)" for x in robot_names]
    data[0].visualize(data[1:], robot_names + robot_names_alt)

if __name__ == "__main__":
    main()