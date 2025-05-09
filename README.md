# ENPM 673 Final Project - Group 2

## Overview

ENPM 673 Final project Repository

---

## Installation

Install yolov5 requirements
```bash
cd yolov5/
pip install -r requirements.txt
```

Go to group2_ws and then perform colcon build

```bash
colcon build  
source install/setup.bash
```

---

## Usage

### 1. Launch the Gazebo World

```bash
ros2 launch enpm673_final_proj enpm673_world.launch.py verbose:=true
```

---

### 2. Run the Main Node

```bash
ros2 run group_2 main_node.py
```

---

### 3. Launch the Horizon Finder Node

```bash
ros2 launch group_2 hf_launch.launch.py
```

---

#### Parameters in Horizon finder node

You can override the following parameters through the command line:

| Parameter                | Default      | Description                                 | Example Override                 |
|--------------------------|--------------|---------------------------------------------|----------------------------------|
| `horizon_y`              | 125          | Initial horizon Y value                     | `horizon_y:=150`                 |
| `hough_threshold`        | 100          | Hough Transform vote threshold              | `hough_threshold:=80`            |
| `hough_angle_resolution` | 0.0174533    | Hough angle resolution in radians (π/180)   | `hough_angle_resolution:=0.0349` |
| `debug`                  | False        | Enable debug mode                           | `debug:=True`                    |

**Example (run in debug mode):**

```bash
ros2 launch group_2 hf_launch.launch.py debug:=True
```

---

- **View final processed display:**

```bash
ros2 run image_view image_view --ros-args --remap image:=/final_display
```

---


- **Run Teleop:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

---

- **View raw camera image:**

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---


## Nodes Overview

- **enpm673_world.launch.py**
  - Launches the Gazebo simulation environment.
  - Publishes camera feed to `/camera/image_raw`.

- **hf_launch.launch.py**
  - Starts the horizon detection pipeline.

- **main_node.py**
  - Implements core logic of the project
  - Publishes processed images to `/final_display`.

---

## Debugging Tips

- Add `verbose:=true` to the Gazebo launch command for detailed simulation logs.
- Use `debug:=True` with the horizon finder node to enable diagnostic outputs.
