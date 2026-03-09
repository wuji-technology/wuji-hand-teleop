#!/usr/bin/env python3
"""Utility script to analyze recorded tianji pose data."""

import pickle
import numpy as np
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_recording(filepath):
    """Load recording from pickle file."""
    with open(filepath, 'rb') as f:
        data = pickle.load(f)
    return data


def analyze_recording(data):
    """Analyze and print statistics about the recording."""
    print("\n" + "="*60)
    print("RECORDING ANALYSIS")
    print("="*60)

    # Basic info
    num_frames = len(data['timestamps'])
    if num_frames == 0:
        print("No data in recording!")
        return

    duration = data['timestamps'][-1] - data['timestamps'][0]
    avg_rate = num_frames / duration if duration > 0 else 0

    print(f"\nBasic Information:")
    print(f"  Total frames: {num_frames}")
    print(f"  Duration: {duration:.2f} seconds")
    print(f"  Average rate: {avg_rate:.1f} Hz")

    # Left arm statistics
    if len(data['left_poses']) > 0:
        left_poses = np.array(data['left_poses'])
        print(f"\nLeft Arm Statistics:")
        print(f"  Number of poses: {len(left_poses)}")

        print(f"\n  Position (meters):")
        print(f"    X: min={left_poses[:, 0].min():.4f}, max={left_poses[:, 0].max():.4f}, mean={left_poses[:, 0].mean():.4f}")
        print(f"    Y: min={left_poses[:, 1].min():.4f}, max={left_poses[:, 1].max():.4f}, mean={left_poses[:, 1].mean():.4f}")
        print(f"    Z: min={left_poses[:, 2].min():.4f}, max={left_poses[:, 2].max():.4f}, mean={left_poses[:, 2].mean():.4f}")

        print(f"\n  Orientation (degrees):")
        print(f"    RX: min={left_poses[:, 3].min():.2f}, max={left_poses[:, 3].max():.2f}, mean={left_poses[:, 3].mean():.2f}")
        print(f"    RY: min={left_poses[:, 4].min():.2f}, max={left_poses[:, 4].max():.2f}, mean={left_poses[:, 4].mean():.2f}")
        print(f"    RZ: min={left_poses[:, 5].min():.2f}, max={left_poses[:, 5].max():.2f}, mean={left_poses[:, 5].mean():.2f}")

    # Right arm statistics
    if len(data['right_poses']) > 0:
        right_poses = np.array(data['right_poses'])
        print(f"\nRight Arm Statistics:")
        print(f"  Number of poses: {len(right_poses)}")

        print(f"\n  Position (meters):")
        print(f"    X: min={right_poses[:, 0].min():.4f}, max={right_poses[:, 0].max():.4f}, mean={right_poses[:, 0].mean():.4f}")
        print(f"    Y: min={right_poses[:, 1].min():.4f}, max={right_poses[:, 1].max():.4f}, mean={right_poses[:, 1].mean():.4f}")
        print(f"    Z: min={right_poses[:, 2].min():.4f}, max={right_poses[:, 2].max():.4f}, mean={right_poses[:, 2].mean():.4f}")

        print(f"\n  Orientation (degrees):")
        print(f"    RX: min={right_poses[:, 3].min():.2f}, max={right_poses[:, 3].max():.2f}, mean={right_poses[:, 3].mean():.2f}")
        print(f"    RY: min={right_poses[:, 4].min():.2f}, max={right_poses[:, 4].max():.2f}, mean={right_poses[:, 4].mean():.2f}")
        print(f"    RZ: min={right_poses[:, 5].min():.2f}, max={right_poses[:, 5].max():.2f}, mean={right_poses[:, 5].mean():.2f}")

    print("\n" + "="*60 + "\n")


def export_to_csv(data, output_path):
    """Export recording data to CSV format."""
    import csv

    with open(output_path, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'left_x', 'left_y', 'left_z', 'left_rx', 'left_ry', 'left_rz',
                      'right_x', 'right_y', 'right_z', 'right_rx', 'right_ry', 'right_rz']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()

        num_frames = len(data['timestamps'])
        for i in range(num_frames):
            row = {'timestamp': data['timestamps'][i]}

            if i < len(data['left_poses']):
                left = data['left_poses'][i]
                row.update({
                    'left_x': left[0], 'left_y': left[1], 'left_z': left[2],
                    'left_rx': left[3], 'left_ry': left[4], 'left_rz': left[5]
                })

            if i < len(data['right_poses']):
                right = data['right_poses'][i]
                row.update({
                    'right_x': right[0], 'right_y': right[1], 'right_z': right[2],
                    'right_rx': right[3], 'right_ry': right[4], 'right_rz': right[5]
                })

            writer.writerow(row)

    print(f"Exported to CSV: {output_path}")


def visualize_recording(data, save_path=None):
    """Create comprehensive visualization of the recording data."""
    if len(data['timestamps']) == 0:
        print("No data to visualize!")
        return

    timestamps = np.array(data['timestamps'])
    # Normalize timestamps to start from 0
    timestamps = timestamps - timestamps[0]

    # Create figure with multiple subplots
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('Tianji Pose Recording Analysis', fontsize=16, fontweight='bold')

    # ========== Left Arm Visualization ==========
    if len(data['left_poses']) > 0:
        left_poses = np.array(data['left_poses'])
        left_times = timestamps[:len(left_poses)]

        # 1. Left arm position over time (3 subplots)
        ax1 = plt.subplot(4, 4, 1)
        ax1.plot(left_times, left_poses[:, 0], 'r-', linewidth=1)
        ax1.set_ylabel('X (m)', fontweight='bold')
        ax1.set_title('Left Arm Position - X', fontweight='bold')
        ax1.grid(True, alpha=0.3)

        ax2 = plt.subplot(4, 4, 2)
        ax2.plot(left_times, left_poses[:, 1], 'g-', linewidth=1)
        ax2.set_ylabel('Y (m)', fontweight='bold')
        ax2.set_title('Left Arm Position - Y', fontweight='bold')
        ax2.grid(True, alpha=0.3)

        ax3 = plt.subplot(4, 4, 3)
        ax3.plot(left_times, left_poses[:, 2], 'b-', linewidth=1)
        ax3.set_ylabel('Z (m)', fontweight='bold')
        ax3.set_title('Left Arm Position - Z', fontweight='bold')
        ax3.grid(True, alpha=0.3)

        # 2. Left arm 3D trajectory
        ax4 = plt.subplot(4, 4, 4, projection='3d')
        scatter = ax4.scatter(left_poses[:, 0], left_poses[:, 1], left_poses[:, 2],
                             c=left_times, cmap='viridis', s=1)
        ax4.set_xlabel('X (m)')
        ax4.set_ylabel('Y (m)')
        ax4.set_zlabel('Z (m)')
        ax4.set_title('Left Arm 3D Trajectory', fontweight='bold')
        plt.colorbar(scatter, ax=ax4, label='Time (s)', shrink=0.5)

        # 3. Left arm orientation over time
        ax5 = plt.subplot(4, 4, 5)
        ax5.plot(left_times, left_poses[:, 3], 'r-', linewidth=1)
        ax5.set_ylabel('RX (deg)', fontweight='bold')
        ax5.set_title('Left Arm Orientation - Roll', fontweight='bold')
        ax5.grid(True, alpha=0.3)

        ax6 = plt.subplot(4, 4, 6)
        ax6.plot(left_times, left_poses[:, 4], 'g-', linewidth=1)
        ax6.set_ylabel('RY (deg)', fontweight='bold')
        ax6.set_title('Left Arm Orientation - Pitch', fontweight='bold')
        ax6.grid(True, alpha=0.3)

        ax7 = plt.subplot(4, 4, 7)
        ax7.plot(left_times, left_poses[:, 5], 'b-', linewidth=1)
        ax7.set_ylabel('RZ (deg)', fontweight='bold')
        ax7.set_title('Left Arm Orientation - Yaw', fontweight='bold')
        ax7.grid(True, alpha=0.3)

        # 4. Left arm velocity (position)
        if len(left_poses) > 1:
            dt = np.diff(left_times)
            dt[dt == 0] = 1e-6  # Avoid division by zero
            vel_x = np.diff(left_poses[:, 0]) / dt
            vel_y = np.diff(left_poses[:, 1]) / dt
            vel_z = np.diff(left_poses[:, 2]) / dt
            vel_mag = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

            ax8 = plt.subplot(4, 4, 8)
            ax8.plot(left_times[:-1], vel_mag, 'purple', linewidth=1)
            ax8.set_ylabel('Speed (m/s)', fontweight='bold')
            ax8.set_title('Left Arm Speed', fontweight='bold')
            ax8.grid(True, alpha=0.3)

    # ========== Right Arm Visualization ==========
    if len(data['right_poses']) > 0:
        right_poses = np.array(data['right_poses'])
        right_times = timestamps[:len(right_poses)]

        # 5. Right arm position over time
        ax9 = plt.subplot(4, 4, 9)
        ax9.plot(right_times, right_poses[:, 0], 'r-', linewidth=1)
        ax9.set_ylabel('X (m)', fontweight='bold')
        ax9.set_title('Right Arm Position - X', fontweight='bold')
        ax9.grid(True, alpha=0.3)

        ax10 = plt.subplot(4, 4, 10)
        ax10.plot(right_times, right_poses[:, 1], 'g-', linewidth=1)
        ax10.set_ylabel('Y (m)', fontweight='bold')
        ax10.set_title('Right Arm Position - Y', fontweight='bold')
        ax10.grid(True, alpha=0.3)

        ax11 = plt.subplot(4, 4, 11)
        ax11.plot(right_times, right_poses[:, 2], 'b-', linewidth=1)
        ax11.set_ylabel('Z (m)', fontweight='bold')
        ax11.set_title('Right Arm Position - Z', fontweight='bold')
        ax11.grid(True, alpha=0.3)

        # 6. Right arm 3D trajectory
        ax12 = plt.subplot(4, 4, 12, projection='3d')
        scatter = ax12.scatter(right_poses[:, 0], right_poses[:, 1], right_poses[:, 2],
                              c=right_times, cmap='plasma', s=1)
        ax12.set_xlabel('X (m)')
        ax12.set_ylabel('Y (m)')
        ax12.set_zlabel('Z (m)')
        ax12.set_title('Right Arm 3D Trajectory', fontweight='bold')
        plt.colorbar(scatter, ax=ax12, label='Time (s)', shrink=0.5)

        # 7. Right arm orientation over time
        ax13 = plt.subplot(4, 4, 13)
        ax13.plot(right_times, right_poses[:, 3], 'r-', linewidth=1)
        ax13.set_ylabel('RX (deg)', fontweight='bold')
        ax13.set_xlabel('Time (s)', fontweight='bold')
        ax13.set_title('Right Arm Orientation - Roll', fontweight='bold')
        ax13.grid(True, alpha=0.3)

        ax14 = plt.subplot(4, 4, 14)
        ax14.plot(right_times, right_poses[:, 4], 'g-', linewidth=1)
        ax14.set_ylabel('RY (deg)', fontweight='bold')
        ax14.set_xlabel('Time (s)', fontweight='bold')
        ax14.set_title('Right Arm Orientation - Pitch', fontweight='bold')
        ax14.grid(True, alpha=0.3)

        ax15 = plt.subplot(4, 4, 15)
        ax15.plot(right_times, right_poses[:, 5], 'b-', linewidth=1)
        ax15.set_ylabel('RZ (deg)', fontweight='bold')
        ax15.set_xlabel('Time (s)', fontweight='bold')
        ax15.set_title('Right Arm Orientation - Yaw', fontweight='bold')
        ax15.grid(True, alpha=0.3)

        # 8. Right arm velocity
        if len(right_poses) > 1:
            dt = np.diff(right_times)
            dt[dt == 0] = 1e-6
            vel_x = np.diff(right_poses[:, 0]) / dt
            vel_y = np.diff(right_poses[:, 1]) / dt
            vel_z = np.diff(right_poses[:, 2]) / dt
            vel_mag = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

            ax16 = plt.subplot(4, 4, 16)
            ax16.plot(right_times[:-1], vel_mag, 'orange', linewidth=1)
            ax16.set_ylabel('Speed (m/s)', fontweight='bold')
            ax16.set_xlabel('Time (s)', fontweight='bold')
            ax16.set_title('Right Arm Speed', fontweight='bold')
            ax16.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Save or show
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Visualization saved to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Analyze tianji pose recordings')
    parser.add_argument('file', type=str, help='Path to the pickle file')
    parser.add_argument('--export-csv', type=str, help='Export data to CSV file')
    parser.add_argument('--quiet', action='store_true', help='Suppress analysis output')
    parser.add_argument('--visualize', action='store_true', help='Show visualization plots')
    parser.add_argument('--save-plot', type=str, help='Save visualization to image file instead of showing')
    parser.add_argument('--no-analyze', action='store_true', help='Skip text analysis, only visualize')

    args = parser.parse_args()

    # Load data
    filepath = Path(args.file)
    if not filepath.exists():
        print(f"Error: File not found: {filepath}")
        return

    print(f"Loading recording: {filepath}")
    data = load_recording(filepath)

    # Analyze
    if not args.quiet and not args.no_analyze:
        analyze_recording(data)

    # Export if requested
    if args.export_csv:
        export_to_csv(data, args.export_csv)

    # Visualize if requested
    if args.visualize or args.save_plot:
        if args.save_plot:
            visualize_recording(data, save_path=args.save_plot)
        else:
            visualize_recording(data)


if __name__ == '__main__':
    main()
