import numpy as np
import open3d as o3d
import cv2
from scipy.ndimage import median_filter
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# -------------------------------
# CONFIGURATION
# -------------------------------
BAG_PATH = 'depth_data'  # ROS bag folder
TOPIC_NAME = '/depth'
FX = 525.0
FY = 525.0
CUBOID_THRESHOLD_MM = 1500
MEDIAN_FILTER_SIZE = 8
DISTANCE_THRESHOLD = 0.01  # RANSAC plane fitting threshold in meters

# -------------------------------
# HELPER FUNCTIONS
# -------------------------------
def depth_to_pointcloud(depth, fx, fy):
    h, w = depth.shape
    cx, cy = w / 2, h / 2
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth
    points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)
    points = points[points[:, 2] > 0.001]
    return points

def extract_largest_plane(points, distance_threshold=0.01):
    """
    Extracts largest plane from point cloud using RANSAC.
    Returns unit normal vector and area of plane (m^2)
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if len(points) < 3:
        return np.array([0,0,1]), 0  # fallback
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    plane_points = points[inliers]
    if len(plane_points) < 3:
        return normal, 0
    hull_pcd = o3d.geometry.PointCloud()
    hull_pcd.points = o3d.utility.Vector3dVector(plane_points)
    mesh, _ = hull_pcd.compute_convex_hull()
    area = mesh.get_surface_area()
    return normal, area

def process_depth_frame(depth_frame):
    # Convert to mm and filter
    depth_mm = depth_frame * 1000.0
    depth_clipped = np.clip(depth_mm, 10, 5000)
    depth_filtered_mm = median_filter(depth_clipped, size=MEDIAN_FILTER_SIZE)
    depth_filtered_m = depth_filtered_mm / 1000.0

    # Focus on cuboid
    cuboid_mask = depth_filtered_mm <= CUBOID_THRESHOLD_MM
    depth_cuboid_m = depth_filtered_m.copy()
    depth_cuboid_m[~cuboid_mask] = 0

    # Convert to point cloud
    points = depth_to_pointcloud(depth_cuboid_m, FX, FY)
    return points

# -------------------------------
# MAIN PROCESS
# -------------------------------
all_normals = []
all_areas = []
frame_numbers = []

with Reader(BAG_PATH) as reader:
    frame_count = 0
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic != TOPIC_NAME:
            continue
        frame_count += 1
        msg = deserialize_cdr(rawdata, connection.msgtype)

        # Decode depth
        if msg.encoding == '32FC1':
            depth_frame = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth_frame = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth_frame = depth_frame.astype(np.float32) / 1000.0
        else:
            continue

        # Process frame
        points = process_depth_frame(depth_frame)
        normal, area = extract_largest_plane(points)

        # Compute angle w.r.t camera (Z-axis)
        angle_deg = np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))

        print(f"Frame {frame_count}: Normal angle = {angle_deg:.2f} deg, Area = {area:.4f} m²")

        all_normals.append(normal)
        all_areas.append(area)
        frame_numbers.append(frame_count)

# -------------------------------
# ESTIMATE ROTATION AXIS
# -------------------------------
N = np.vstack(all_normals)
_, _, Vt = np.linalg.svd(N)
rotation_axis = Vt[-1]  # axis vector

print("Estimated rotation axis (camera frame):", rotation_axis)

# -------------------------------
# SAVE OUTPUTS
# -------------------------------
# 1. Table of frame | angle | area
import pandas as pd
df = pd.DataFrame({
    'Frame': frame_numbers,
    'Normal_Angle_deg': [np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0))) for n in all_normals],
    'Visible_Area_m2': all_areas
})
df.to_csv('cuboid_face_angles_areas.csv', index=False)
print("Saved frame-angle-area table to 'cuboid_face_angles_areas.csv'")

# 2. Rotation axis text file
np.savetxt('rotation_axis.txt', rotation_axis, fmt='%.6f')
print("Saved rotation axis to 'rotation_axis.txt'")
