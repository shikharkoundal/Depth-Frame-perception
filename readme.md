# 3D Cuboid Rotation Estimation from Depth Frames

**Author:** Shikhar Koundal

---

### Project Overview

This project focuses on estimating the **rotation angle** and **rotation axis** of a **cuboidal box** using **depth frames** captured by a depth camera.  
It uses Python and Open3D for 3D point cloud processing, plane segmentation, and rotation analysis.  
The project is implemented in a ROS2 environment where depth frames are extracted from a `.db3` bag file.

---

### Problem Statement

Given a ROS2 bag file containing depth frames of a cuboidal box rotating around its central axis, the main objectives were:

1. Identify the largest visible face of the cuboid in each frame.  
2. Estimate the normal angle of that face relative to the camera.  
3. Calculate the visible surface area of the face.  
4. Determine the rotation axis of the cuboid with respect to the camera frame.


The main challenges included filtering the background, handling noisy depth data, and performing stable geometric estimation across multiple frames.

---

### Approach

**1. Depth Frame Inspection**  
Depth frames were extracted from the bag and visually inspected.  
The cuboid appeared darker in the depth image (closer to the camera), helping isolate it from the background.

**2. Depth Frame Processing**  
- Converted depth values from meters to millimeters for better precision.  
- Applied a median filter to reduce noise.  
- Masked out the background to focus only on the cuboid region.

**3. Point Cloud Generation**  
Each filtered depth frame was converted into a **3D point cloud** using the camera’s intrinsic parameters (`fx, fy, cx, cy`).  
This allowed geometric analysis in 3D space.

**4. Plane Segmentation**  
- Used **RANSAC** to detect the largest planar face.  
- Extracted the plane normal vector and convex hull to calculate visible area.  
- Calculated the normal angle using:



θ = arccos( (n ⋅ c) / (|n||c|) )



where `n` is the plane normal and `c = [0, 0, 1]` is the camera direction.![Uploading Screenshot from 2025-10-18 22-42-03.png…]()


**5. Rotation Axis Estimation**  
- Found cross products between normals of consecutive frames to estimate instantaneous rotation axes.  
- Averaged all axes to get the final rotation axis in the camera frame.

**6. Visualization (Optional)**  
Used Open3D to visualize:
- Point clouds of each frame  
- Plane normals  
- The estimated rotation axis

---

### Algorithm Summary

1. Load depth frames from ROS2 bag.  
2. Convert and filter depth data.  
3. Generate 3D point cloud.  
4. Detect the largest plane using RANSAC.  
5. Compute normal angle and visible area.  
6. Estimate rotation axis using cross products.  
7. Save outputs as CSV and text files.



### Results

<img width="475" height="171" alt="Screenshot from 2025-10-18 22-42-03" src="https://github.com/user-attachments/assets/c6288e51-3970-46e5-90ca-18e347f3f655" />
| Frame | Normal Angle (°) | Area (m²) |
|--------|------------------|-----------|
| 1 | 64.92 | 1.7272 |
| 2 | 14.15 | 2.0779 |
| 3 | 35.00 | 2.1848 |
| 4 | 125.55 | 1.8411 |
| 5 | 29.99 | 1.3493 |
| 6 | 49.02 | 2.0930 |
| 7 | 48.78 | 2.5824 |

**Estimated Rotation Axis (camera frame):**  
`[ 0.99927848, -0.02534483, 0.02828721 ]`

Saved outputs:  
- `cuboid_face_angles_areas.csv`  
- `rotation_axis.txt`



### Tools and Libraries Used

- Python 3  
- ROS2 (Humble)  
- Open3D  
- NumPy  
- SciPy  
- Matplotlib  

---

### How to Run

1. Place your ROS2 bag file (for example, `depth.db3`) in the project directory.  
2. Run the main script:
bash
 python3 cuboid_rotation_estimation.py


3. The output will generate:

   * A CSV file with frame-wise angle and area.
   * A text file containing the estimated rotation axis.
4. (Optional) Run `visualize.py` to see 3D visualization.

---

### Summary

This project demonstrates:

* Converting depth data into 3D point clouds.
* Detecting and analyzing planar surfaces using RANSAC.
* Estimating orientation and rotation axis of a 3D object.

It combines computer vision and geometric reasoning to estimate motion in 3D space — a useful approach for robotics and perception systems.

---

### Author

**Shikhar Koundal**
B.Tech Final Year | Robotics & Computer Vision Enthusiast
[LinkedIn](https://linkedin.com/in/shikharkoundal) | [GitHub](https://github.com/sk1133)

```
