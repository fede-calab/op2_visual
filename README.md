# OP2 Visual Tracking Package

## Project Description

The `op2_visual` is an advanced computer vision package designed for precise ball tracking and camera calibration, leveraging ROS (Robot Operating System) and OpenCV technologies. This package provides robust solutions for visual tracking in robotic and motion analysis applications.

## System Requirements

### Hardware
- Compatible webcam or camera
- Computer running ROS
- Calibration chessboard

### Software
- ROS Noetic or Melodic
- OpenCV 4.x
- C++17 compatible compiler
- Custom `ball_msgs` message package

## Installation

1. Clone the repository:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url>
   ```

2. Install dependencies:
   ```bash
   rosdep install op2_visual
   ```

3. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Camera Calibration Node

### Purpose
Removes lens distortion by calculating camera intrinsic parameters using a precise chessboard calibration method.

### Usage
```bash
rosrun op2_visual camera_calibration_node --camera-name <camera_identifier>
```

#### Calibration Parameters
- `--camera-name`: Unique identifier for camera configuration
- `--no-gui`: Suppress graphical interface

### Calibration Process
1. Capture multiple chessboard images
2. Detect internal corners
3. Compute camera matrix and distortion coefficients
4. Generate calibration parameter file

#### Chessboard Specifications
- Internal Corner Pattern: 8x6
- Square Size: 25mm
- Recommended Images: 10-20 different angles

## Ball Tracking Node

### Functionality
Implements advanced ball detection and tracking using:
- HSV color-based detection
- ArUco marker positioning
- Real-time velocity calculation

### Usage
```bash
rosrun op2_visual op2_visual_node --camera-name <camera_identifier>
```

#### Node Parameters
- `--camera-name`: Specify camera configuration
- `--no-gui`: Disable visual output
- `offset_ratio`: Adjust marker reference point

### Tracking Features
- Color-based ball detection
- Marker-based positioning
- Velocity computation
- Distance tracking
- CSV data logging

### ROS Topic
- **Topic**: `/ball_topic`
- **Message Type**: `ball_msgs/ball`
- **Published Data**: 
  - Velocity (cm/s)
  - Relative distance (cm)

## Data Outputs

### Tracking Logs
- Location: `data/ball_tracking_data.csv`
- Columns: Timestamp, Distance, Velocity, Marker Positions

### Calibration Files
- Location: `config/<camera_name>_params.yml`
- Contains: Camera matrix, distortion coefficients

## Troubleshooting

1. Verify camera connection
2. Check ROS master status
3. Validate calibration files
4. Adjust HSV color range for ball detection

## Performance Considerations

- Recommended frame rate: 30 FPS
- Optimal lighting conditions
- Clear color contrast for ball detection

## Contributing

Contributions are welcome. Please follow:
- Standard ROS development practices
- Consistent code formatting
- Comprehensive unit testing

## License

[Add specific license information]

## Contact

**Author**: Calabrese Federico
**Project**: OP2 Visual Tracking