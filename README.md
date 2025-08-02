# Target-Based LiDAR-Camera Calibration

A LiDAR-Camera extrinsic calibration tool using rectangular calibration boards for accurate spatial relationship determination between LiDAR and camera sensors.

## Overview

This software provides an accurate method to determine the spatial relationship between LiDAR and camera sensors using rectangular calibration boards. The current version offers a user-friendly, semi-automated calibration process with manual assistance features.

## Prerequisites

### System Requirements
- Linux (Ubuntu 18.04+ recommended)
- CMake 3.10+

### Dependencies

Install the required dependencies:

```bash
# Install basic dependencies
sudo apt install cmake build-essential python3-opencv libopencv-dev libboost-all-dev libpcl-dev software-properties-common

# Add GTSAM PPA and install
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## Installation

### Build from Source

1. Clone the repository:
```bash
git clone https://github.com/KEEPSLAMDUNK/target-based-calibration.git
cd target-based-calibration
```

2. Create build directory and compile:
```bash
mkdir build && cd build
cmake ..
make -j8
```

## Usage

### Quick Demo

Run the calibration tool with the provided demo data:

```bash
./bin/lidar_camera_calibrator ../data/data-velo-sim
```

## Calibration Workflow

![Calibration Process](doc/a9907fce31514ad7b62549bb8872ccf43067.png)

The calibration process follows these main steps:
1. **Data Collection** - Capture synchronized LiDAR and camera data with calibration board
2. **Data Selection** - Choose high-quality image frames for calibration
3. **Configuration** - Set camera intrinsics and calibration board parameters
4. **Corner Detection** - Extract calibration board corners from images and point clouds
5. **Calibration** - Compute extrinsic parameters
6. **Validation** - Verify results through visualization

## Step-by-Step Calibration Guide

### 1. Data Collection (rosbag)

- Hold the calibration board at various angles during data collection
- Change the board position 3-5 times to ensure comprehensive coverage
- Capture data across the overlapping field of view between LiDAR and camera
- Ensure the calibration board is clearly visible in both sensors

### 2. Data Selection

Select 3-5 high-quality image frames with clear calibration board visibility:

![Sample Image 1](doc/55d61adf-9a12-45ed-b9cb-29651ca57e59.jpeg)
![Sample Image 2](doc/66d8e21f-8cb6-4750-a453-d03005e80bce.jpeg)
![Sample Image 3](doc/e9f9bfc1-cc06-44e0-a7b4-246038d235e4.jpeg)

### 3. Configuration Setup

Modify the configuration file (`config.json`) with:
- Camera intrinsic parameters
- Distortion coefficients
- Calibration board size

![Configuration Example 1](doc/d5ecfd46-49ee-4aaa-bba8-0131a39cf9b6.png)
![Configuration Example 2](doc/d112c018-2122-4a0b-a4f3-fefc2181ebd6.png)

### 4. Extrinsic Calibration Process

#### 4.1 Launch Calibration Tool

1. Open the calibration software
2. Specify the root directory containing calibration data

![Launch Interface](doc/e2cd60cd-0ee1-4ce0-bbe2-c13a7d2e0a1b.png)

#### 4.2 Corner Extraction

**Image Corner Detection:**
- Calibration board corners are automatically detected in images
- If no automatic detection occurs, manually select corners:
  - Click "Start" to begin manual selection
  - Click "Finish" to complete selection

![Image Corner Detection](doc/c0f824a4-9029-4721-9d48-c25f5ae71a05.png)

**Point Cloud Corner Extraction:**
- Manually filter the calibration board from the point cloud
- Click "Extract" to complete corner extraction

![Point Cloud Filtering](doc/037789a2-4529-4bd8-9cdd-ca95e72a55c8.png)
![Point Cloud Extraction](doc/e3d8fd98-4b6e-4128-90d5-bef081cafdbc.png)

### 5. Results and Validation

After completing corner extraction for all datasets:

1. Click the "Calibrate" button to perform calibration
2. View calibration results in the terminal output
3. Visualize the results through:
   - Point cloud projection onto images
   - Colored point cloud visualization
4. Calibration results are automatically saved to the configuration file

![Calibration Results 1](doc/de71298d-7272-467b-998f-5d0cc2305621.png)
![Calibration Results 2](doc/643d8a13-24b2-4219-8885-27aedaccd42f.png)

## Results

The calibration process provides:
- **Extrinsic Parameters**: Translation and rotation matrices between LiDAR and camera
- **Visualization**: Point cloud overlaid on camera images
- **Validation Metrics**: Reprojection errors and calibration accuracy statistics

![Result Visualization 1](doc/7a104b19-89ed-46b0-8bd9-2bc0b4ff98b7.png)
![Result Visualization 2](doc/f2734f5b-e02d-4ee8-8327-8538b60c56d8.png)

## Troubleshooting

### Common Issues

- **Poor corner detection**: Ensure adequate lighting and clear calibration board visibility
- **Insufficient data**: Collect more poses with varied orientations

## Contributing

Contributions are welcome! Please feel free to submit issues and enhancement requests.

## TODO

- [ ] Code refactoring
- [ ] Better documentation/video tutorials  
- [ ] Further algorithm improvements

## Acknowledgments

This project is built upon open-source work:

- [HITSZ-NRSL/lidar_camera_calibrator](https://github.com/HITSZ-NRSL/lidar_camera_calibrator)
- [beltransen/velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration)