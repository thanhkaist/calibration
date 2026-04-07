# Robot Vision Calibration

A Python toolkit for calibrating robot vision systems, covering camera intrinsics, hand-eye (eye-in-hand) extrinsics, laser line scanners, and turntables. The library is built on OpenCV, NumPy, and `math3d`, and targets Python 2.7 / OpenCV 2.x environments.

---

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Dependencies](#dependencies)
- [Modules](#modules)
  - [Camera Calibration](#camera-calibration)
  - [Hand-Eye Calibration](#hand-eye-calibration)
  - [Laser Calibration](#laser-calibration)
  - [Turntable Calibration](#turntable-calibration)
  - [UDP Linescan Plotter](#udp-linescan-plotter)
- [Quick-Start Examples](#quick-start-examples)
- [Output Files](#output-files)
- [References](#references)

---

## Overview

This repository provides tools for the calibration pipeline commonly used in industrial robot-vision applications:

```
Camera intrinsics  ──►  Hand-eye extrinsics  ──►  Laser plane  ──►  Turntable axis
   (camera_calibration)     (hand_eye_calibration)   (laser_calibration)  (turntable_calibration)
```

Each step produces calibration parameters (stored as NumPy `.npy` files or OpenCV XML/YAML files) that can be consumed by the next step or directly by the acquisition pipeline in `udp_linescan_plotter.py`.

---

## Repository Structure

```
calibration/
├── camera_calibration/                  # Monocular camera intrinsic calibration
│   ├── camera_calibrator.py             # Main calibration class + OpenCV GUI
│   ├── camera_calibrator_base.py        # MonoCalibrator base (chessboard detection, ROS-style)
│   ├── webcamcapture.py                 # OpenCV webcam capture backend
│   ├── araviscapture.py                 # GigE/Aravis camera capture backend
│   ├── atc4capture.py                   # ATC4 camera capture backend (TCP socket)
│   └── pvlibcapture.py                  # PvLib camera capture backend
│
├── camera_calibration_cpp_prototype/    # C++ prototype using OpenCV
│   └── CMakeLists.txt
│
├── hand_eye_calibration/                # Eye-in-hand (AX = XB) calibration
│   ├── hand_eye_calibration.py          # High-level HandEyeCalibration class
│   ├── tsai_lenz_calibration.py         # Tsai–Lenz 1989 method
│   ├── park_martin_calibration.py       # Park–Martin 1994 method
│   ├── aim_tool_pose_generator.py       # Random pose generator for data collection
│   └── park_martin_test.py              # Test script for Park–Martin
│
├── laser_calibration/                   # Laser line scanner calibration
│   ├── stepped_calibration_object/
│   │   └── calibrator.py               # Stepped-block calibration object workflow
│   ├── sal3d_calibration_object/
│   │   └── laser_calibration.py        # SAL3D calibration object workflow
│   ├── plane_fit_3d_svd.py             # SVD-based 3-D plane fitting utility
│   ├── restore.py                       # Restore/reconstruct 3-D from scan
│   └── restore_cpp/                     # C++ restore helper
│
├── turntable_calibration/
│   └── turntable_calibration.py         # Turntable axis / center / speed calibration
│
├── udp_linescan_plotter.py              # Real-time linescan viewer + point-cloud capture
└── README.md
```

---

## Dependencies

| Package | Purpose |
|---|---|
| Python 2.7 | Runtime (uses `Queue`, `cv2.cv`, `print` without parentheses in some places) |
| OpenCV 2.x (`cv2`, `cv2.cv`) | Image processing, chessboard detection, `solvePnP` |
| NumPy | Numerical arrays and linear algebra |
| [`math3d`](https://pypi.org/project/math3d/) | 3-D transforms, orientations, and vectors |
| SciPy | Loading `.mat` files (benchmark datasets) |
| matplotlib / pylab | Real-time linescan plotting and point-cloud saving |

Install Python dependencies:

```bash
pip install numpy scipy matplotlib math3d
```

OpenCV 2.x must be installed separately (e.g., via your system package manager or a pre-built wheel).

---

## Modules

### Camera Calibration

**Location:** `camera_calibration/`

Calibrates a monocular camera using a chessboard calibration pattern. Produces the camera matrix **K** and distortion coefficients **d**.

#### Key Classes

| Class | File | Description |
|---|---|---|
| `MonoCalibrator` | `camera_calibrator_base.py` | Core calibration logic – collects frames, checks coverage, and runs `cv2.calibrateCamera` |
| `CameraCalibration` | `camera_calibrator.py` | Thread-safe wrapper (grabber thread + consumer thread) |
| `OpenCVCalibration` | `camera_calibrator.py` | Adds an interactive OpenCV GUI on top of `CameraCalibration` |

#### Capture Backends

| Module | Class | Hardware |
|---|---|---|
| `webcamcapture.py` | — | Any OpenCV-compatible webcam (`cv2.VideoCapture`) |
| `araviscapture.py` | `AravisCapture` | GigE Vision cameras via the Aravis library |
| `atc4capture.py` | `ATC4Capture` | Proprietary ATC4 frame grabber (TCP socket) |
| `pvlibcapture.py` | `PvLibCapture` | Allied Vision cameras via PvLib/Vimba SDK |

#### Interactive GUI Keyboard Shortcuts

| Key | Action |
|---|---|
| `c` | Calibrate camera (when enough samples are collected) |
| `s` | Save calibration to disk |
| `u` | Toggle undistorted preview |
| `q` / `Esc` | Quit |

#### Usage Example

```python
import cv2
from camera_calibration.camera_calibrator import OpenCVCalibration
from camera_calibration.camera_calibrator_base import ChessboardInfo

capture = cv2.VideoCapture(0)          # or AravisCapture(), ATC4Capture(), etc.
boards = [ChessboardInfo(8, 6, 0.0245)]  # cols, rows, square_size_in_metres
calibrator = OpenCVCalibration(capture, boards, output_dir="./params/")

import time
while True:
    time.sleep(1)
```

#### C++ Prototype

A CMake-based C++ prototype lives in `camera_calibration_cpp_prototype/`. Build with:

```bash
cd camera_calibration_cpp_prototype
mkdir build && cd build
cmake ..
make
./camera_calibration
```

---

### Hand-Eye Calibration

**Location:** `hand_eye_calibration/`

Solves the classic **AX = XB** equation to find the fixed transformation between a robot flange and a camera mounted on it (eye-in-hand configuration). The calibration object must be fixed in the robot's base frame and fully visible by the camera.

**Inputs:**
- A set of ≥ 3 pose pairs: *(flange-to-base transform, object-to-camera transform)*

**Output:**
- `sensor_in_flange` — the `math3d.Transform` describing the camera pose in the flange frame

#### Implemented Algorithms

| Class | File | Algorithm |
|---|---|---|
| `TsaiLenzCalibrator` | `tsai_lenz_calibration.py` | Tsai & Lenz, *IEEE Trans. Robotics and Automation*, 1989 |
| `ParkMartinCalibrator` | `park_martin_calibration.py` | Park & Martin, *IEEE Trans. Robotics and Automation*, 1994 |

The high-level `HandEyeCalibration` class in `hand_eye_calibration.py` wraps both solvers and adds chessboard-based pose estimation via `cv2.solvePnP`.

#### Pose Data Collection

`AimToolPoseGenerator` generates random robot poses that keep the camera aimed at a calibration target, useful for automatically collecting a diverse set of pose pairs:

```python
from hand_eye_calibration.aim_tool_pose_generator import AimToolPoseGenerator

gen = AimToolPoseGenerator(
    centre_pos=m3d.Vector([1.12, 0.0, 1.27]),   # robot workspace centre
    aim_pos=m3d.Vector([2.2, 0.2, 0.8]),          # calibration target position
)
pose = gen()   # returns a math3d.Transform
```

#### Usage Example

```python
import numpy as np
from hand_eye_calibration.tsai_lenz_calibration import TsaiLenzCalibrator

# pose_pairs: numpy array of shape (N, 2) with math3d.Transform elements
# pose_pairs[i][0] = flange-to-base transform
# pose_pairs[i][1] = object-to-camera transform
pose_pairs = np.load('eye2hand_test_pose_pairs.npy')

tlc = TsaiLenzCalibrator(pose_pairs)
X = tlc.sensor_in_flange    # math3d.Transform
print(X)
```

---

### Laser Calibration

**Location:** `laser_calibration/`

Calibrates a laser line scanner (structured-light profilometer) to determine the laser plane equation in the camera coordinate system.

Two calibration object workflows are provided:

#### 1. Stepped Calibration Object (`stepped_calibration_object/calibrator.py`)

A multi-step block (5 height levels) is used. The interactive workflow:

1. **Plane 1 & 5** — left-click on 4 fiducial marks → right-click to compute homography H₁.
2. **Plane 2 & 4** — same as above → H₂.
3. **Plane 3** — same → H₃, then `solvePnP` to find the object pose.
4. **Laser line** — left-click 10 times to define limits on each step → right-click to fit the laser plane.

Outputs are saved via `_save_calibration()`.

```python
import numpy as np
from laser_calibration.stepped_calibration_object.calibrator import Calibrator

cm = np.load('params/cm.npy')
dc = np.load('params/dc.npy')
fullframe  = np.load('params/frame-full.npy')
laserline  = np.load('params/frame-cog.npy')   # centre-of-gravity encoded linescan

subpixel = 6   # COG bit depth
lc = Calibrator(fullframe, subpixel, laserline=laserline, camera_matrix=cm, dist_coeffs=dc)
```

#### 2. SAL3D Calibration Object (`sal3d_calibration_object/laser_calibration.py`)

Uses a four-point planar target; receives live linescan data over UDP. Interactive workflow via matplotlib:

- **Middle-click** — add a correspondence point (up to 4)
- **Right-click** (after 4 points) — run `calibrate_laser()` and save

```python
import numpy as np
from laser_calibration.sal3d_calibration_object.laser_calibration import LaserCalibration

cm = np.load('params/cm.npy')
dc = np.load('params/dc.npy')
lc = LaserCalibration(cm, dc, save_path='./params')
```

#### Plane Fitting Utility

`laser_calibration/plane_fit_3d_svd.py` demonstrates SVD-based plane fitting:

```python
from numpy import *
A = random.randn(100, 3)    # point cloud
u, s, vh = linalg.svd(A - A.mean(axis=0))
normal = vh.conj().T[:, -1]  # plane normal
```

---

### Turntable Calibration

**Location:** `turntable_calibration/turntable_calibration.py`

Calibrates a motorised turntable by tracking a chessboard pattern placed on its surface. Determines:
- **Rotation center** — 3-D position of the turntable axis in the camera frame
- **Rotation axis** — unit vector of the turntable axis
- **Angular speed** — radians per second

#### Algorithm

1. Capture 3 timestamped frames while the turntable rotates.
2. Detect chessboard corners and solve for the board's translation vector (tvec) at each position.
3. Fit a circle through the 3 translation vectors using `find_rotation_point_and_axis()`.

#### Usage Example

```python
import numpy as np
from turntable_calibration.turntable_calibration import TurntableCalibrator, ATC4Capture

cm   = np.load('params/cm.npy')
dc   = np.load('params/dc.npy')
h    = np.load('params/homography.npy')
rvec = np.load('params/rvec_laser_calib_obj.npy')
tvec = np.load('params/tvec_laser_calib_obj.npy')

cap = ATC4Capture()
tc = TurntableCalibrator(cap, cm, dc, h, rvec, tvec, save_path='./params')
tc.calibrate(debug=True)
tc.save_calibration()
```

---

### UDP Linescan Plotter

**Location:** `udp_linescan_plotter.py`

Real-time matplotlib tool that:
1. Continuously receives linescan (COG) data over UDP (port 8888, 2048 pixels × 16-bit).
2. Undistorts and perspective-transforms the linescan into the laser plane frame.
3. Optionally accumulates scanlines during a timed scan session.
4. Transforms all scanlines into the turntable reference frame using the turntable calibration.
5. Saves the resulting point cloud as a PLY file.

#### GUI Controls

| Control | Action |
|---|---|
| **Middle-click** (×2) | Define horizontal scan range |
| **Right-click** | Toggle scanning start / stop |
| **Start** button | Start scan (requires range to be set) |
| **Stop** button | Stop scan |
| **Save** button | Transform scanlines and save `pc.ply` |

#### Usage Example

```python
import numpy as np
from udp_linescan_plotter import UDPLinescanPlotter

cm       = np.load('params/cm.npy')
dc       = np.load('params/dc.npy')
h        = np.load('params/homography.npy')
rvec     = np.load('params/rvec_laser_calib_obj.npy')
tvec     = np.load('params/tvec_laser_calib_obj.npy')
angspeed = np.load('params/turntable_angspeed.npy')
center   = np.load('params/turntable_center.npy')
axis     = np.load('params/turntable_axis.npy')

lp = UDPLinescanPlotter(cm, dc, h, rvec, tvec, angspeed, center, axis)
```

---

## Output Files

| File | Module | Description |
|---|---|---|
| `params/cm.npy` | Camera calibration | Camera matrix (3×3 float32) |
| `params/dc.npy` | Camera calibration | Distortion coefficients (1×5 float32) |
| `params/camera_matrix.xml` | Camera calibration | Camera matrix in OpenCV XML format |
| `params/distortion_coefficients.xml` | Camera calibration | Distortion coefficients in OpenCV XML format |
| `params/homography.npy` | Laser calibration | Image-to-object homography (3×3) |
| `params/rvec_laser_calib_obj.npy` | Laser calibration | Rotation vector (3×1) of calibration object |
| `params/tvec_laser_calib_obj.npy` | Laser calibration | Translation vector (3×1) of calibration object |
| `params/laser_plane.txt` | Laser calibration | Laser plane normal vector |
| `params/point_on_laser_plane.txt` | Laser calibration | A point on the laser plane |
| `params/turntable_center.npy` | Turntable calibration | 3-D rotation center |
| `params/turntable_axis.npy` | Turntable calibration | 3-D rotation axis unit vector |
| `params/turntable_angspeed.npy` | Turntable calibration | Angular speed (rad/s) |
| `pc.ply` | UDP linescan plotter | Reconstructed 3-D point cloud (ASCII PLY) |

---

## References

1. **Tsai, R.Y. and Lenz, R.K.** (1989). *A new technique for fully autonomous and efficient 3D robotics hand/eye calibration.* IEEE Transactions on Robotics and Automation, 5(3), 345–358. [DOI:10.1109/70.34770](https://doi.org/10.1109/70.34770)

2. **Park, F.C. and Martin, B.J.** (1994). *Robot Sensor Calibration: Solving AX = XB on the Euclidean Group.* IEEE Transactions on Robotics and Automation, 10(5), 717–721. [DOI:10.1109/70.326576](https://doi.org/10.1109/70.326576)

3. **Bradski, G.** (2000). *The OpenCV Library.* Dr. Dobb's Journal of Software Tools.

---

*Authors: Lars Tingelstad (NTNU/IPK), Morten Lind (SINTEF Raufoss Manufacturing AS). License: GPLv3.*
