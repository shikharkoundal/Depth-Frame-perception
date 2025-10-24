# 3D Cuboid Rotation Estimation from

# Depth Frames

## Author: Shikhar Koundal

## Purpose

The purpose of this project is to develop a method to estimate the **rotation angle and axis of a
cuboidal object** from depth frames captured by a camera. The project demonstrates practical
skills in **3D point cloud processing, plane segmentation, rotation analysis, and geometric
reasoning** —all highly relevant in robotics, industrial automation, and computer vision.

## Problem Statement

Given a ROS2 bag file containing depth frames of a cuboidal box rotating around its central axis,
the objectives are:

1. Identify the **largest visible face** in each frame.
2. Estimate the **normal angle** of that face relative to the camera.
3. Calculate the **visible area** of the face in square meters.
4. Determine the **axis of rotation** of the cuboid with respect to the camera frame.

Challenges include isolating the cuboid from the background, filtering noisy depth data, and
performing accurate geometric analysis to infer orientation over time.

## Approach

**Step 1: File Preparation and Initial Inspection**

- All ROS2 bag files were placed into the workspace.
- Depth frames were visually inspected. Darker regions corresponded to the cuboid, while
    the background was lighter.
- This observation guided masking and filtering, ensuring analysis focused only on the
    cuboid.

```
Figure 1: Example depth frame showing the cuboid as the darker region in the center.
```
**Step 2: Depth Frame Processing**


- Depth values were **converted from meters to millimeters** for better precision with
    near-camera points.
- Applied a **median filter** to smooth the depth frame and reduce noise.
- Masked out the background, keeping only points corresponding to the cuboid.

**Step 3: Point Cloud Generation**

- Converted the filtered depth frame into a 3D point cloud using camera intrinsics (focal
    lengths FX, FY and image center).
- This allowed geometric analysis in 3D space.

**Step 4: Plane Segmentation**

- The largest planar face was identified using RANSAC.
- Extracted the plane normal (n=[𝑎,𝑏,𝑐]) and convex hull to calculate visible area.
- Plane centers were also stored for rotation axis estimation.
    Normal angle calculation formula:

```
𝜽=𝐚𝐫𝐜𝐜𝐨𝐬⁡(
```
### 𝐧⋅𝐜

### ∥𝐧∥∥𝐜∥

### )

```
Where:
```
- 𝑛 _= plane normal vector_
- 𝑐 _= camera normal vector (typically_ [ 0 , 0 , 1 ] _)_
- 𝜃 _= angle between camera and plane normal_

**Step 5: Rotation Axis Estimation**

- Calculated cross products of consecutive normals to estimate instantaneous rotation
    axes:
       axis𝑖=normalize(n𝑖×n𝑖+ 1 )
- Averaged these vectors to obtain the final rotation axis relative to the camera frame:
    axis=

### 1

### 𝑁

### ∑

```
𝑖
```
```
axis𝑖
```
- This method assumes the cuboid rotates smoothly around a fixed axis.

**Step 6: Optional Verification (3D Visualization)**

- Although visualization was not required, Open3D was used to verify the point cloud and
    plane normals.
- Cuboid points, normals, and the rotation axis were visualized to confirm alignment with
    the expected geometry.

## Algorithm Summary

1. Load Depth Frames: Extract frames from ROS2 bag using rosbags.
2. Convert & Filter: Convert depth to millimeters, apply median filter, and mask cuboid
    region.
3. Depth to Point Cloud: Generate 3D points using camera intrinsics.


4. Largest Plane Detection: Use RANSAC → compute normal vector, convex hull, and
    visible area.
5. Normal Angle Calculation: Use the formula above.
6. Rotation Axis Estimation: Cross products of consecutive normals → average.
7. Save Results: Output normal angles, visible areas, and rotation axis.

## Results

Frame 1: Normal angle = 64.92 deg, Area = 1.7272 m²
Frame 2: Normal angle = 14.15 deg, Area = 2.0779 m²
Frame 3: Normal angle = 35.00 deg, Area = 2.1848 m²
Frame 4: Normal angle = 125.55deg, Area = 1.8411 m²
Frame 5: Normal angle = 29.99 deg, Area = 1.3493 m²
Frame 6: Normal angle = 49.02 deg, Area = 2.0930 m²
Frame 7: Normal angle = 48.78 deg, Area = 2.5824 m²
Estimated rotation axis (camera frame): [ 0.99927848 -0.02534483 0.02828721]
Saved frame-angle-area table to 'cuboid_face_angles_areas.csv'
Saved rotation axis to 'rotation_axis.txt'

```
Figure 3: Side-by-side visualization of depth frame (masked) and 3D cuboid point cloud.
```
## Key Highlights

- Efficient **depth-to-3D conversion** and masking of the cuboid.
- Robust **plane segmentation** to extract normals and visible areas.
- Simple and reliable **rotation axis estimation**.
- Optional visualization confirmed the geometric correctness of extracted parameters.


