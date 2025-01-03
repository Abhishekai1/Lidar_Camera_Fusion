import numpy as np
import os
import cv2
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D 

# Function to load ground truth LiDAR data from KITTI dataset
def load_kitti_lidar_data(lidar_file):
    """
    Loads ground truth LiDAR data (3D point cloud) from a KITTI bin file.

    Parameters:
    - lidar_file: Path to the LiDAR .bin file containing the 3D point cloud data.

    Returns:
    - points: 3D LiDAR points (N, 4), where the last column is intensity.
    """
    points = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)
    return points[:, :3]  # Use only x, y, z coordinates (ignoring intensity)

# Example usage
lidar_file = '/home/ubuntu18/Documents/kitti2bag-master/2011_09_26/2011_09_26_drive_0014_sync/velodyne_points/data/0000000000.bin'  # Path to a specific .bin file
gt_points = load_kitti_lidar_data(lidar_file)

# Display a few points
print("Ground Truth LiDAR Points:\n{}".format(gt_points[:5]))

# Save for evaluation
np.save('Ground_points.npy', gt_points)

# Plotting the 3D LiDAR points
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(gt_points[:, 0], gt_points[:, 1], gt_points[:, 2], c='r', marker='.')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

