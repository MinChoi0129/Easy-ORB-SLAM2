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
apt-get update && apt-get install -y \
    software-properties-common build-essential cmake git wget unzip \
    libglew-dev libegl1-mesa-dev libgl1-mesa-dev libwayland-dev \
    libxkbcommon-dev xorg-dev libx11-dev libpng-dev libjpeg-dev \
    libtiff-dev libavcodec-dev libavformat-dev libswscale-dev \
    libavutil-dev libeigen3-dev libopencv-dev libepoxy-dev

add-apt-repository -y ppa:mhier/libboost-latest
apt-get update
apt-get install -y libboost1.83-all-dev

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin
cd /tmp/Pangolin
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
make install
```

### Build

```bash
cd your_workspace
git clone https://github.com/MinChoi0129/Easy-ORB-SLAM2.git && cd Easy-ORB-SLAM2
cd build (build folder already exists)
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
