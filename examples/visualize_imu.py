#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox

# EXAMPLE USAGE: python3 visualize_imu_with_range.py /path/to/imu.txt "Drone1 IMU"
# ./visualize_imu.py /media/sgarimella34/hercules-collect/raw_data_hercules/test2_2uav2ugv_calib_752x480/Drone1/imu.txt "Drone1 simIMU"
# ./visualize_imu.py /media/sgarimella34/hercules-collect/raw_data_hercules/test2_2uav2ugv_calib_752x480/Drone1/synthetic_imu.txt "Drone1 syntheticIMU"

def visualize_imu_with_range(imu_file, title):
    # Load data
    data = np.loadtxt(imu_file)
    t = data[:, 0]
    ax_vals = data[:, 1]; ay_vals = data[:, 2]; az_vals = data[:, 3]
    gx_vals = data[:, 4]; gy_vals = data[:, 5]; gz_vals = data[:, 6]

    # Create figure & axes
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    plt.subplots_adjust(top=0.90, bottom=0.18)  # room for suptitle & textboxes

    # Set window title if backend supports it
    try:
        fig.canvas.manager.set_window_title(title)
    except Exception:
        pass

    # Set figure suptitle
    fig.suptitle(title, fontsize=14)

    # Plot linear acceleration
    ax1.plot(t, ax_vals, label='ax')
    ax1.plot(t, ay_vals, label='ay')
    ax1.plot(t, az_vals, label='az')
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.set_title('Linear Acceleration (NED)')
    ax1.legend(loc='upper right')
    ax1.grid(True)

    # Plot angular velocity
    ax2.plot(t, gx_vals, label='gx')
    ax2.plot(t, gy_vals, label='gy')
    ax2.plot(t, gz_vals, label='gz')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Angular Velocity (NED)')
    ax2.legend(loc='upper right')
    ax2.grid(True)

    # TextBoxes for start/end time
    axbox_start = plt.axes([0.15, 0.05, 0.25, 0.04])
    tb_start = TextBox(axbox_start, 'Start t', initial=f"{t.min():.6f}")

    axbox_end = plt.axes([0.55, 0.05, 0.25, 0.04])
    tb_end = TextBox(axbox_end, 'End t', initial=f"{t.max():.6f}")

    def update(_):
        """Callback to update x-limits when either box is submitted."""
        try:
            t0 = float(tb_start.text)
            t1 = float(tb_end.text)
            if t0 < t1:
                ax1.set_xlim(t0, t1)
                ax2.set_xlim(t0, t1)
                fig.canvas.draw_idle()
        except ValueError:
            pass  # ignore invalid input

    tb_start.on_submit(update)
    tb_end.on_submit(update)

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Visualize IMU data with interactive time‐range selection."
    )
    parser.add_argument(
        'imu_file',
        help="Path to imu.txt (columns: t ax ay az gx gy gz)"
    )
    parser.add_argument(
        'title',
        help="Title for this plot/window"
    )
    args = parser.parse_args()
    visualize_imu_with_range(args.imu_file, args.title)
