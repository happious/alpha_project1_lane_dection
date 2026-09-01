# Alpha Project 1: OpenCV-based Lane Detection

> OpenCV 기반 영상처리 기법을 학습하고, 차선 검출부터 Bird's Eye View 및 RANSAC 기반 곡선 차선 추정까지 단계적으로 구현한 프로젝트입니다.

## 📑 Overview

- [Introduction](#-introduction)
- [Key Features](#-key-features)
- [Pipeline](#-pipeline)
  - [1. Sliding Window Lane Detection](#1-sliding-window-lane-detection)
  - [2. MORAI + RANSAC Curve Fitting](#2-morai--ransac-curve-fitting)
- [Project Files](#-project-files)
- [Results](#-results)
- [Tech Stack](#-tech-stack)
- [Development Environment](#-development-environment)
- [How to Run](#-how-to-run)
- [What I Learned](#-what-i-learned)

---

## 🚀 Introduction

Alpha Project 1에서는 **전통적인 Computer Vision 기반 차선 인식**을 주제로 OpenCV의 기본 영상처리 기법부터 차선 검출 알고리즘까지 단계적으로 구현했습니다.

1학기에는 HSV 색 검출, Sliding Window, Filter & Edge Detection, ORB Feature Matching 등을 학습하고 구현했으며,  
하계 프로젝트에서는 **Perspective Transformation을 이용한 Bird's Eye View**와 **RANSAC 기반 Curve Fitting**을 적용하여 곡선 형태의 차선을 추정하도록 확장했습니다.

RANSAC 단계에서는 MORAI Simulator의 카메라 영상을 ROS Topic으로 수신하고, 차선 검출 결과를 ROS `Path` 메시지로 생성하는 구조까지 구성했습니다.

---

## 🔑 Key Features

- **HSV 기반 색상 영역 검출**
  - BGR 이미지를 HSV Color Space로 변환
  - Threshold 기반 특정 색상 영역 분리

- **Sliding Window 기반 차선 검출**
  - Binary Image에서 좌·우 차선 픽셀 탐색
  - Polynomial Fitting을 이용한 차선 곡선 추정
  - 검출된 차선 영역을 원본 영상에 시각화

- **Bird's Eye View**
  - Perspective Transformation을 이용한 Top View 변환
  - 원근 왜곡을 줄여 차선 구조 분석을 단순화

- **RANSAC 기반 Curve Fitting**
  - `scikit-learn`의 `RANSACRegressor` 활용
  - 다항식 Feature를 이용한 좌·우 차선 Curve Fitting
  - Outlier의 영향을 줄이고 안정적인 차선 모델 추정
  - 한쪽 차선 검출이 부족한 경우 기존 Lane Width를 이용해 보완

- **MORAI Simulator + ROS 연동**
  - `/image_jpeg/compressed` 카메라 Topic 구독
  - 검출된 좌·우 차선의 중앙을 ROS `Path` 메시지로 변환
  - `/lane_path` Topic으로 Publish

- **Filter / Edge Detection 비교**
  - Average, Gaussian Blur
  - Roberts Cross, Prewitt, Sobel, Scharr

- **ORB Feature Matching**
  - ORB 특징점 추출
  - BFMatcher + Hamming Distance 기반 이미지 매칭

---

## 🛠 Pipeline

프로젝트에서는 차선 검출 방법을 단계적으로 발전시키며 두 가지 주요 파이프라인을 구현했습니다.

1. **Sliding Window 기반 차선 검출**
2. **MORAI Simulator + RANSAC Curve Fitting 기반 차선 추정**

---

### 1. Sliding Window Lane Detection

초기 차선 검출 단계에서는 도로 영상을 Bird's Eye View로 변환한 뒤, Binary Image에서 Sliding Window 방식으로 좌·우 차선을 추적했습니다.

![Sliding Window Lane Detection Pipeline](src/pipeline_sliding_window.png)

#### Pipeline

```text
Bird's Eye View & Binary Lane Extraction
                  ↓
Sliding Window Lane Detection
                  ↓
Lane Area Visualization
```

#### ① Bird's Eye View & Binary Lane Extraction

Perspective Transformation을 이용해 도로 영상을 위에서 내려다본 형태의 **Bird's Eye View**로 변환합니다.  
이후 HLS Lightness Channel과 Threshold를 이용해 차선 영역을 Binary Image로 추출합니다.

```text
Camera Image
     ↓
Perspective Transform
     ↓
Bird's Eye View
     ↓
HLS Lightness Threshold
     ↓
Binary Lane Image
```

#### ② Sliding Window Lane Detection

Binary Image의 하단 Histogram을 이용해 좌·우 차선의 시작 위치를 찾고, 여러 개의 Window를 아래에서 위로 이동시키며 차선 Pixel을 탐색합니다.

탐색된 좌·우 차선 Point를 이용해 Polynomial Fitting을 수행하여 차선의 형태를 추정합니다.

```text
Binary Lane Image
        ↓
Histogram
        ↓
Left / Right Start Point
        ↓
Sliding Window Search
        ↓
Lane Pixel Collection
        ↓
Polynomial Fitting
```

#### ③ Lane Area Visualization

추정된 좌측 차선과 우측 차선 사이의 영역을 채운 뒤, Inverse Perspective Transform을 적용하여 원본 카메라 시점으로 복원합니다.

최종적으로 원본 영상 위에 검출된 주행 차선 영역을 Overlay하여 시각화합니다.

```text
Left / Right Lane
        ↓
Lane Area Fill
        ↓
Inverse Perspective Transform
        ↓
Original Image Overlay
```

---

### 2. MORAI + RANSAC Curve Fitting

하계 프로젝트에서는 MORAI Simulator와 ROS를 연동하고, 기존 Sliding Window 방식에서 확장하여 **Bird's Eye View + RANSAC Curve Fitting** 기반의 차선 추정 구조를 구현했습니다.

최종 RANSAC 구조는 **`lane_detection.py` + `util.py`**를 중심으로 동작합니다.

![MORAI RANSAC Curve Fitting Pipeline](src/pipline_ransac.png)

#### Pipeline

```text
MORAI Camera Image
        ↓
ROI Masking
        ↓
Bird's Eye View Transform
        ↓
Lane Binarization
        ↓
Lane Pixel Reconstruction
        ↓
RANSAC Curve Fitting & Lane Estimation
        ↓
Lane Center Path Generation
        ↓
/lane_path Publish
```

#### ① ROI Masking

MORAI Simulator의 `/image_jpeg/compressed` 카메라 영상을 수신한 뒤, 전체 영상에서 차선 검출에 필요한 도로 영역만 ROI로 설정합니다.

ROI 외부 영역을 제거하여 이후 영상처리 과정에서 불필요한 정보를 줄입니다.

#### ② Bird's Eye View Transform

`util.py`의 `BEVTransform`을 이용해 카메라 영상을 위에서 내려다보는 형태의 **Bird's Eye View**로 변환합니다.

원근 효과를 줄여 차선의 위치와 곡률을 보다 단순한 형태로 처리할 수 있도록 합니다.

#### ③ Lane Binarization

Bird's Eye View 영상에서 흰색 및 노란색 차선 영역을 분리하여 하나의 Binary Lane Mask를 생성합니다.

```text
BEV Image
    ↓
White Lane Mask
    +
Yellow Lane Mask
    ↓
Binary Lane Image
```

#### ④ Lane Pixel Reconstruction

Binary Image에서 차선에 해당하는 Pixel을 추출하고, 이미지 좌표 `(u, v)`를 Bird's Eye View 기준의 차선 좌표 `(x, y)`로 변환합니다.

```text
Binary lane pixels (u, v)
            ↓
Coordinate Reconstruction
            ↓
Reconstructed lane points (x, y)
```

이렇게 복원된 좌표는 이후 RANSAC Curve Fitting의 입력 Point로 사용됩니다.

#### ⑤ RANSAC Curve Fitting & Lane Estimation

`util.py`의 `CURVEFit` 클래스에서 `scikit-learn`의 `RANSACRegressor`를 이용하여 좌·우 차선을 각각 추정합니다.

차선 Point의 X 좌표를 Polynomial Feature로 변환한 뒤 RANSAC을 적용하여 Outlier의 영향을 줄이고 차선을 잘 설명하는 Curve를 추정합니다.

```text
Lane Points
    ↓
Polynomial Features
    ↓
Left / Right Point Selection
    ↓
RANSAC Curve Fitting
    ↓
Outlier Rejection
    ↓
Left / Right Lane Estimation
```

직선 구간뿐 아니라 곡선 구간에서도 동일한 구조로 좌·우 차선을 추정할 수 있도록 구성했습니다.

#### ⑥ Lane Center Path Generation

RANSAC으로 추정된 좌·우 차선의 중앙값을 계산하여 차량이 따라갈 **중심 경로**를 생성합니다.

```python
center_y = 0.5 * (y_pred_l + y_pred_r)
```

생성된 중심 경로는 ROS `Path` 메시지로 변환되어 `/lane_path` Topic으로 Publish됩니다.

```text
Left Lane     Right Lane
     \         /
      \       /
       Center Path
           ↓
      /lane_path
```

이를 통해 영상 기반 차선 검출 결과를 이후 차량 주행 제어 알고리즘에서 사용할 수 있는 경로 정보로 연결했습니다.

---


## 📁 Project Files

RANSAC 및 MORAI 차선 검출 관련 주요 파일은 다음과 같습니다.

| File | Description |
|---|---|
| `lane_detection.py` | **MORAI 기반 RANSAC 차선 검출 메인 ROS 노드** |
| `util.py` | `BEVTransform`, `CURVEFit`, Lane Path 생성 및 Pure Pursuit 관련 공통 모듈 |
| `morai_roi.py` | MORAI 카메라 영상에서 Mouse Callback으로 ROI 좌표를 확인하는 보조 노드 |
| `sliding_window_lane_detection.py` | Sliding Window + Polynomial Fitting 기반 초기 차선 검출 코드 |


---

## 📊 Results

프로젝트를 통해 다음 기능을 구현했습니다.

| Task | Result |
|---|---|
| HSV Color Detection | 특정 색상 영역 검출 |
| Sliding Window | 좌·우 차선 영역 추적 |
| Bird's Eye View | 도로 영상 Top View 변환 |
| RANSAC Line Fitting | 직접 구현한 RANSAC 기반 직선 차선 추정 실험 |
| RANSAC Curve Fitting | `RANSACRegressor` 기반 좌·우 곡선 차선 추정 |
| MORAI Camera | ROS CompressedImage 기반 카메라 영상 수신 |
| Lane Path | 좌·우 차선 중앙 경로를 `/lane_path`로 Publish |
| Edge Detection | 다양한 Edge Filter 비교 |
| Feature Matching | ORB 기반 특징점 매칭 |

특히 하계 프로젝트에서는 기존 차선 검출을 **MORAI Simulator + Bird's Eye View + RANSAC Curve Fitting** 구조로 확장하고, 검출 결과를 ROS Path로 연결하는 과정을 구현했습니다.

---

## 💻 Tech Stack

| Category | Technology |
|---|---|
| Language | Python 3 |
| Computer Vision | OpenCV |
| Numerical Computing | NumPy |
| Robust Regression | scikit-learn `RANSACRegressor` |
| Robot Middleware | ROS1 (`rospy`) |
| Simulator | MORAI Simulator |
| ROS Messages | `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `morai_msgs` |
| Symbolic Math | SymPy *(초기 RANSAC 실험 코드)* |

---

## ⚙️ Development Environment

현재 남아 있는 코드에는 구버전 NumPy / scikit-learn API가 포함되어 있어 최신 Python 환경보다 **ROS1 기반 레거시 환경**에서 재현하는 것을 권장합니다.

### Recommended Reproduction Environment

| Package | Recommended Version |
|---|---|
| OS | Ubuntu 20.04 |
| ROS | ROS Noetic |
| Python | 3.8.x |
| NumPy | 1.19.5 |
| OpenCV | 4.5.x |
| scikit-learn | 0.24.2 |
| SymPy | 1.8 *(초기 실험 코드 실행 시)* |

> 위 버전은 당시 프로젝트에서 `pip freeze`로 기록한 정확한 환경이 아니라,  
> 현재 보관된 소스 코드의 API 사용 방식에 맞춘 **재현용 권장 환경**입니다.

### Compatibility Note

초기 Sliding Window 코드에는 다음과 같은 구버전 NumPy API가 포함되어 있습니다.

```python
np.int(...)
```

따라서 최신 NumPy에서는 해당 코드를 그대로 실행할 경우 호환성 오류가 발생할 수 있습니다.

또한 RANSAC Curve Fitting 코드에서는 구버전 scikit-learn API인 다음 인자를 사용합니다.

```python
RANSACRegressor(
    base_estimator=...,
    loss='absolute_loss'
)
```

현재 코드를 수정하지 않고 재현하려는 경우 위의 권장 버전을 사용하는 것이 편리합니다.

### Python Dependencies

ROS 환경 외 Python 패키지는 다음과 같이 설치할 수 있습니다.

```bash
pip3 install numpy==1.19.5
pip3 install opencv-python==4.5.5.64
pip3 install scikit-learn==0.24.2
```

초기 `ransac_move.py`까지 실행하려면 SymPy를 추가합니다.

```bash
pip3 install sympy==1.8
```

ROS의 `rospy`, `cv_bridge`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`와 MORAI의 `morai_msgs`는 ROS 환경에서 별도로 구성되어 있어야 합니다.

---

## ▶️ How to Run

### 1. Sliding Window 코드 실행

`sliding_window_lane_detection.py`는 `xycar_track1.mp4`를 직접 읽는 독립 실행형 코드입니다.

```text
project/
├── sliding_window_lane_detection.py
└── xycar_track1.mp4
```

두 파일을 같은 디렉터리에 둔 뒤 실행합니다.

```bash
python3 sliding_window_lane_detection.py
```

> 저장된 코드에는 `cv2.waitKey(0)`가 사용되어 있어 프레임마다 키 입력을 기다립니다.  
> 연속 영상으로 확인하려면 필요에 따라 `cv2.waitKey(1)`로 변경할 수 있습니다.

---

### 2. MORAI RANSAC Lane Detection 실행

MORAI 기반 RANSAC 차선 검출은 단일 Python 파일 실행이 아니라 **MORAI Simulator + ROS1 + ROS Package** 환경에서 동작합니다.

대표적인 파일 구성은 다음과 같습니다.

```text
catkin_ws/
└── src/
    └── beginner_tutorials/
        ├── scripts/
        │   ├── lane_detection.py
        │   ├── util.py
        │   ├── moria+roi.py
        │   └── 카메라실행노드.py
        └── sensor/
            └── sensor_params.json
```

`lane_detection.py`는 코드 내부에서 다음 ROS Package를 찾습니다.

```python
currentPath = rospkg.RosPack().get_path("beginner_tutorials")
```

그리고 다음 파일에서 카메라 파라미터를 읽습니다.

```text
beginner_tutorials/sensor/sensor_params.json
```

#### Step 1. ROS 환경 실행

```bash
roscore
```

다른 Terminal에서는 Workspace를 Source합니다.

```bash
source ~/catkin_ws/devel/setup.bash
```

#### Step 2. MORAI Simulator 실행

MORAI Simulator와 ROS 통신 환경을 실행하고 카메라 Topic이 생성되는지 확인합니다.

```bash
rostopic list
```

차선 검출 노드는 다음 Topic을 사용합니다.

```text
/image_jpeg/compressed
```

#### Step 3. 카메라 영상 확인

카메라 영상 수신 여부를 먼저 확인할 수 있습니다.

```bash
rosrun beginner_tutorials 카메라실행노드.py
```

카메라 영상이 정상적으로 OpenCV 창에 표시되면 ROS Camera Topic이 정상적으로 들어오는 상태입니다.

#### Step 4. ROI 좌표 확인

필요하면 `morai_roi.py`를 이용해 MORAI 카메라 영상에서 ROI 좌표를 확인합니다.

```bash
rosrun beginner_tutorials moria+roi.py
```

영상 위의 Point를 클릭하여 좌표를 확인하고 차선 검출 ROI 설정에 사용합니다.

#### Step 5. HSV / ROI Parameter 설정

현재 보관된 `lane_detection.py`에는 다음 값이 Placeholder 상태로 남아 있습니다.

```python
self.lower_wlane = np.array([0,0,0])
self.upper_wlane = np.array([0,0,0])

self.lower_ylane = np.array([0,0,0])
self.upper_ylane = np.array([0,0,0])

self.crop_pts = np.array([[[0,0],[0,0],[0,0],[0,0]]])
```

따라서 실제 차선 검출 전에 환경에 맞는 다음 값을 설정해야 합니다.

- White Lane HSV Range
- Yellow Lane HSV Range
- ROI 4 Point 좌표

#### Step 6. Main Lane Detection Node 실행

파라미터 설정 후 메인 노드를 실행합니다.

```bash
rosrun beginner_tutorials lane_detection.py
```

정상적으로 실행되면 다음 과정이 반복됩니다.

```text
/image_jpeg/compressed
        ↓
ROI Mask
        ↓
Bird's Eye View
        ↓
Lane Binarization
        ↓
Lane Point Reconstruction
        ↓
RANSAC Curve Fitting
        ↓
Left / Right Lane
        ↓
/lane_path
```

ROS Topic으로 생성된 Lane Path는 다음 명령으로 확인할 수 있습니다.

```bash
rostopic echo /lane_path
```

---

## 📚 What I Learned

- OpenCV를 이용한 기본 Image Processing Pipeline 구성
- HSV / Threshold / Edge Detection의 차이와 활용 방법
- Perspective Transformation과 Homography의 원리
- Sliding Window 기반 차선 픽셀 탐색
- RANSAC을 이용한 Outlier Robust Line / Curve Estimation
- `scikit-learn RANSACRegressor`를 이용한 다항 차선 Curve Fitting
- MORAI Simulator 카메라 영상의 ROS Topic 처리
- Bird's Eye View 좌표와 영상 좌표 사이의 변환
- 좌·우 차선 중앙을 ROS `Path` 메시지로 생성하는 과정
- 영상처리 결과를 차량 주행 시스템에서 사용할 수 있는 경로 정보로 연결하는 구조
- ORB Descriptor와 Feature Matching 구조
