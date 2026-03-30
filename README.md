# Easy to Install ORB-SLAM2 with Map(save/load) & OpenCV4

### Overview

ORB-SLAM2 is a complete SLAM system for monocular, stereo, and RGB-D cameras.

**Key Features:**
- Built on OpenCV 4, compatible with modern systems
- Real-time visual SLAM
- **Map Save/Load** - Save generated maps and reload them later
- **Relocalization** - Estimate camera pose against saved maps

### Dependencies

```bash
sudo apt-get install libopencv-dev libeigen3-dev libboost-all-dev
```

**Main Packages:**
- OpenCV 4
- Eigen3
- Boost 1.83.0 (filesystem, system, serialization)
- Pangolin (visualization)

### Build

```bash
mkdir build && cd build
cmake ..
make -j8
```

### Running Examples

**1. Real-time SLAM with Webcam**
```bash
./slam_webcam
```

**2. SLAM from Video File**
```bash
./slam_video
```

**3. Relocalization with Webcam**
```bash
./relocalization_webcam
```

**4. Relocalization from Video**
```bash
./relocalization_video
```

### Output

- `KeyFrameTrajectory.txt` - Camera trajectory
- `data/map/` - Saved map files
