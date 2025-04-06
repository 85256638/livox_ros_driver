#!/usr/bin/env python
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2, Imu
import ros_numpy
import numpy as np
from datetime import datetime
import os
import matplotlib.pyplot as plt  # For color mapping

# ===== Editable Parameters =====
FRAME_TIME = 1.0  # Frame time in seconds (adjust as needed)
OUTPUT_DIR = "/home/changning/pointclouds/"  # Directory to save PCD and IMU files (ensure it exists)
# ===============================

class LivoxRecorder:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('livox_recorder', anonymous=True)

        self.point_buffer = []  # Buffer to store point cloud data
        self.start_time = rospy.Time.now().to_sec()  # Record the start time
        self.imu_data = []  # Buffer to store IMU data

        # Format start time in the desired format
        self.start_time_human = datetime.fromtimestamp(self.start_time).strftime("%Y-%m-%d_%H:%M:%S")
        
        # Initial IMU file path with "_ongoing" tag
        self.imu_file_path = os.path.join(OUTPUT_DIR, f"imu_data_{self.start_time_human}_ongoing.csv")
        
        # Ensure the output directory exists
        if not os.path.exists(OUTPUT_DIR):
            os.makedirs(OUTPUT_DIR)
            rospy.loginfo(f"Created directory: {OUTPUT_DIR}")

        # Initialize IMU log file
        with open(self.imu_file_path, 'w') as imu_file:
            imu_file.write("epoch_timestamp,human_readable_time,orientation_x,orientation_y,orientation_z,orientation_w,"
                           "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
                           "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z\n")
        rospy.loginfo(f"Initialized IMU log file: {self.imu_file_path}")

        # Subscribe to the Livox topics
        rospy.Subscriber('/livox/lidar', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/livox/imu', Imu, self.imu_callback)

    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 message to a structured numpy array
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

        # Extract x, y, z, and intensity fields
        points = np.zeros((pc_np.shape[0], 4), dtype=np.float32)
        points[:, 0] = pc_np['x']
        points[:, 1] = pc_np['y']
        points[:, 2] = pc_np['z']
        points[:, 3] = pc_np['intensity']  # Assuming intensity is a valid field

        # Add points to the buffer
        self.point_buffer.append(points)

        # Check if the frame time has elapsed
        current_time = rospy.Time.now().to_sec()
        if current_time - self.start_time >= FRAME_TIME:
            # Save the accumulated point cloud as a PCD file
            self.save_pcd()

            # Reset the buffer and start time
            self.point_buffer = []
            self.start_time = current_time

    def imu_callback(self, msg):
        # Extract IMU data and append it to the buffer
        epoch_timestamp = rospy.Time.now().to_sec()
        human_readable_time = datetime.fromtimestamp(epoch_timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")
        imu_entry = [
            epoch_timestamp,
            human_readable_time,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ]
        self.imu_data.append(imu_entry)

    def save_pcd(self):
        # Combine all points in the buffer
        if len(self.point_buffer) == 0:
            rospy.logwarn("Point buffer is empty. Skipping PCD save.")
            return

        all_points = np.vstack(self.point_buffer)

        # Create an Open3D PointCloud object
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(all_points[:, :3])  # Use only x, y, z for geometry

        # Apply a heatmap to the intensity values
        intensities = all_points[:, 3]
        max_intensity = intensities.max() if intensities.max() > 0 else 1.0
        normalized_intensities = intensities / max_intensity  # Normalize to [0, 1]

        # Map normalized intensities to a heatmap (viridis)
        cmap = plt.get_cmap('viridis')
        colors = cmap(normalized_intensities)[:, :3]  # Extract RGB (drop alpha)
        pc.colors = o3d.utility.Vector3dVector(colors)

        # Generate a timestamped filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        filename = f"{OUTPUT_DIR}livox_{timestamp}.pcd"

        # Save the PCD file
        o3d.io.write_point_cloud(filename, pc)
        rospy.loginfo(f"Saved PCD file with heatmap colors: {filename}")

        # Save IMU data to the log file
        self.save_imu()

    def save_imu(self):
        if len(self.imu_data) == 0:
            rospy.logwarn("IMU buffer is empty. Skipping IMU save.")
            return

        # Append IMU data to the CSV file
        with open(self.imu_file_path, 'a') as imu_file:
            for imu_entry in self.imu_data:
                imu_file.write(','.join(map(str, imu_entry)) + '\n')

        # Clear the IMU buffer after saving
        self.imu_data = []
        rospy.loginfo(f"Saved IMU data to {self.imu_file_path}")

    def finalize_imu_file(self):
        # Get the end time in the desired format
        end_time_human = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        
        # Rename the IMU file to include both start and end times
        finalized_path = os.path.join(
            OUTPUT_DIR, f"imu_data_{self.start_time_human}_to_{end_time_human}.csv"
        )
        os.rename(self.imu_file_path, finalized_path)
        rospy.loginfo(f"Renamed IMU file to: {finalized_path}")

    def run(self):
        try:
            rospy.spin()
        finally:
            self.finalize_imu_file()

if __name__ == "__main__":
    recorder = LivoxRecorder()
    recorder.run()
