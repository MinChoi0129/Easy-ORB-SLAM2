# Easy to Install ORB-SLAM2 with Map(save/load) & OpenCV4

[한국어](#한국어) | [English](#english)

---
아래의 README 내용은 LLM으로 작성되었습니다.\
The content of README below is written by LLM.

---

## 한국어

### 개요

ORB-SLAM2는 모노큘러, 스테레오, RGB-D 카메라를 지원하는 완전한 SLAM 시스템입니다.

**주요 특징:**
- OpenCV 4 기반으로 최신 시스템에서도 안정적으로 동작
- 실시간 시각적 SLAM 수행
- **맵 저장 및 로드 기능** - 생성된 맵을 저장한 후 나중에 불러올 수 있음
- **재위치결정(Relocalization)** - 저장된 맵을 기준으로 카메라 위치 재인식

### 필수 의존성

```bash
sudo apt-get install libopencv-dev libeigen3-dev libboost-all-dev
```

**주요 패키지:**
- OpenCV 4
- Eigen3
- Boost 1.83.0 (filesystem, system, serialization)
- Pangolin (시각화)

### 빌드

```bash
mkdir build && cd build
cmake ..
make -j8
```

### 예제 실행

**1. 웹캠 기반 SLAM (실시간)**
```bash
./slam_webcam
```

**2. 비디오 파일 SLAM**
```bash
./slam_video
```

**3. 웹캠에서 재위치결정**
```bash
./relocalization_webcam
```

**4. 비디오에서 재위치결정**
```bash
./relocalization_video
```

### 출력

- `KeyFrameTrajectory.txt` - 카메라 궤적
- `data/map/` - 저장된 맵 파일

---

## English

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
