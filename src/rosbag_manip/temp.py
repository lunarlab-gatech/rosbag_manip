# import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Load the CSV files
# file1 = "/home/dbutterfield3/Research/rosbag_manip/outputs/_hercules_node_Husky2_ground_truth_odom_local.csv"
# file2 = "/home/dbutterfield3/Research/rosbag_manip/outputs/_hercules_node_Husky2_ground_truth_odom_localNOISE.csv"

# df1 = pd.read_csv(file1, header=None, names=['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
# df2 = pd.read_csv(file2, header=None, names=['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])


# df1 = df1.apply(pd.to_numeric, errors='coerce').dropna()
# df2 = df2.apply(pd.to_numeric, errors='coerce').dropna()

# # Extract positions
# x1, y1, z1 = df1['x'], df1['y'], df1['z']
# x2, y2, z2 = df2['x'], df2['y'], df2['z']

# # Plotting
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# ax.plot(x1, y1, z1, label='Trajectory 1', color='blue')
# ax.plot(x2, y2, z2, label='Trajectory 2', color='red')

# # Set labels
# ax.set_title("Trajectory Comparison")
# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# ax.legend()

# # ----- Force equal axis scale -----
# # Combine all points to find global bounds
# all_x = pd.concat([x1, x2])
# all_y = pd.concat([y1, y2])
# all_z = pd.concat([z1, z2])

# # Find the center and max range
# x_center = (all_x.max() + all_x.min()) / 2
# y_center = (all_y.max() + all_y.min()) / 2
# z_center = (all_z.max() + all_z.min()) / 2

# max_range = max(
#     all_x.max() - all_x.min(),
#     all_y.max() - all_y.min(),
#     all_z.max() - all_z.min()
# ) / 2

# # Set limits to center ± max_range
# ax.set_xlim(x_center - max_range, x_center + max_range)
# ax.set_ylim(y_center - max_range, y_center + max_range)
# ax.set_zlim(z_center - max_range, z_center + max_range)
# # -----------------------------------

# plt.tight_layout()
# plt.show()

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# Load CSV files
file1 = "/media/dbutterfield3/T73/hercules_datasets/V1.2/ausenv_test1_checkpoints_CSLAM_2UAVUGV_prunedExtract/_hercules_node_Husky1_ground_truth_odom_local.csv"
file2 = "/media/dbutterfield3/T73/hercules_datasets/V1.2/ausenv_test1_checkpoints_CSLAM_2UAVUGV_prunedExtract/_hercules_node_Husky1_ground_truth_odom_localNOISE.csv"

df1 = pd.read_csv(file1, header=None, names=['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
df2 = pd.read_csv(file2, header=None, names=['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])

df1 = df1.apply(pd.to_numeric, errors='coerce').dropna()
df2 = df2.apply(pd.to_numeric, errors='coerce').dropna()

# Extract positions
x1, y1, z1 = df1['x'], df1['y'], df1['z']
x2, y2, z2 = df2['x'], df2['y'], df2['z']

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x1, y1, z1, label='GT Odometry', color='blue')
ax.plot(x2, y2, z2, label='Noisy Odometry (with random start)', color='red')

# Draw orientation axes (X = red, Y = green, Z = blue)
arrow_interval = 5000
arrow_length = 5  # meters

def draw_axes(df, label_prefix=""):
    for i in range(0, len(df), arrow_interval):
        row = df.iloc[i]
        pos = np.array([row['x'], row['y'], row['z']])
        quat = np.array([row['qw'], row['qx'], row['qy'], row['qz']])
        rot = R.from_quat(quat, scalar_first=True)

        # Define unit vectors for X, Y, Z in local frame
        x_axis = rot.apply([1, 0, 0])
        y_axis = rot.apply([0, 1, 0])
        z_axis = rot.apply([0, 0, 1])

        # Plot X axis (red)
        ax.quiver(*pos, *x_axis, length=arrow_length, color='r', normalize=True, linewidth=0.8)
        # Plot Y axis (green)
        ax.quiver(*pos, *y_axis, length=arrow_length, color='g', normalize=True, linewidth=0.8)
        # Plot Z axis (blue)
        ax.quiver(*pos, *z_axis, length=arrow_length, color='b', normalize=True, linewidth=0.8)

# Draw for both trajectories
draw_axes(df1)
draw_axes(df2)

# Set labels
ax.set_title("Trajectory Comparison with Full Orientation")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.legend()

# Equal scale for all axes
all_x = pd.concat([x1, x2])
all_y = pd.concat([y1, y2])
all_z = pd.concat([z1, z2])
x_center = (all_x.max() + all_x.min()) / 2
y_center = (all_y.max() + all_y.min()) / 2
z_center = (all_z.max() + all_z.min()) / 2
max_range = max(all_x.max() - all_x.min(), all_y.max() - all_y.min(), all_z.max() - all_z.min()) / 2
ax.set_xlim(x_center - max_range, x_center + max_range)
ax.set_ylim(y_center - max_range, y_center + max_range)
ax.set_zlim(z_center - max_range, z_center + max_range)

plt.tight_layout()
plt.show()
