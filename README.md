# Beekeeping_arm
# README

### Description

Robotics project aiming to automate the handling of beehives using a 6-degree-of-freedom robotic arm.

Developed within the Agricultural Robotics Laboratory of Qingdao City University, the project enables the robotic arm to perceive its environment and manipulate different elements of a beehive.

The system uses a camera to detect the covers and frames of the hive through various image processing techniques. The information obtained is then used to guide the arm's movements and carry out different manipulation sequences.

### Main Features

- Frame detection: Identification of the outer rectangle (cover) via image processing using HSV masks and morphological operations

- Iterative alignment: Automatic alignment of the J0 (vertical), J1 (angular), and J3 (horizontal center) axes through visual servoing

- Bar detection: Identification of bars via horizontal projection after image rectification through perspective transformation

- Calculated descent: Descent of the robot to X cm from the frame with kinematic compensation on the J0 and J3 axes

- Bar inspection: Navigation to each bar and inspection by rotating the J5 axis to +90° and -90°

- Return home: Automatic return of all joints to 0 with verification of final positions

- Command file interface: Allows the sequence to be modified without recompiling or editing the main program

- Built-in diagnostics: CHECK function to display the complete system status (positions, Z, descent state)

### Prerequisites

#### Python Dependencies

```
pip install opencv-python numpy rclpy
```

#### ROS2

The program is designed to work with ROS2 and requires the following topics:

- Publishers:
  - /arm/joint_position (Float64MultiArray) - Arm joint positions
  - /wrist/joint_position (Float64MultiArray) - Wrist joint positions

- Subscribers:
  - /arm/joint_state (Float64MultiArray) - Arm state
  - /wrist/joint_state (Float64MultiArray) - Wrist state

#### Hardware

- Camera (tested with index 2, configurable in the code)
- Robot with 6 axes (J0 to J5)
- Hive covers and frames

### User Interface

#### Keyboard Commands

| Key | Action |
|--------|--------|
| a | Vertical alignment (J0) |
| g | Angular alignment (J1) |
| v | Centered vertical alignment (J3) |
| s | Full sequence + inspection |
| b | Cancel freeze (ROI) |
| d | Enable/disable debug mode |
| c | Display 3D coordinates |
| f | Load a command file |
| h | Display help |
| q | Quit |

#### Command File

The program accepts command files to automate sequences. This approach allows the definition of the robotic task to be separated from its software implementation.

##### Available Commands

| Command | Syntax | Description |
|----------|---------|-------------|
| V | V v0 v1 v2 v3 v4 v5 | Speeds of joints J0..J5 |
| H | H 1 | Return home (all to 0) |
| I | I | Initialization (waiting for ROS positions) |
| A | A | Full alignment + Z measurement |
| D | D height_cm | Descent to X cm |
| B | B 1,2,3,4 | Navigation to bars |
| W | W seconds | Wait X seconds |
| P | P j0 j1 j2 j3 j4 j5 | Absolute position |
| M | M dj0 dj1 dj2 dj3 dj4 dj5 | Relative movement |
| Z | Z | Measure Z distance |
| R | R angle | Rotate J5 to X degrees |
| C | C | Display current positions |
| CHECK | CHECK | Full diagnostics |
| S | S file | Save configuration |
| L | L file | Load configuration |
| HELP | HELP | Display help |
| QUIT | QUIT | Quit |

##### Command File Example

```
# Initialization
I
W 1

# Full alignment
A
W 2

# Position check
C
W 1

# Descent to 2 cm
D 2
W 2

# Navigation to bars 1, 2, and 3
B 1,2,3
W 2

# Return home
H 1

# Final check
C
```

### Full Sequence

#### Phase 1: Alignments

1. Angular alignment (J1): Aligns the frame horizontally
2. Vertical alignment (J0): Centers the frame vertically
3. Centered vertical alignment (J3): Centers the frame horizontally
4. Z measurement: Calculates the distance to the frame
5. J0 center recording: Saves the central position

#### Phase 2: Descent

1. Distance calculation: Based on the Z measurement
2. J2 descent: 2/3 of the total distance
3. J4 descent: 1/3 of the total distance
4. Compensations: J0 (horizontal) and J3 (vertical)

#### Phase 3: Navigation + Inspection

For each requested bar:

1. J0 movement: Positioning on the bar
2. J2 raise: +200 mm
3. J5 rotation to +90 degrees: Inspection side 1
4. Wait: 5 seconds
5. J5 rotation to -90 degrees: Inspection side 2
6. Wait: 5 seconds
7. J5 return to 0 degrees
8. J2 descent: Return to initial position

#### Phase 4: Return Home

1. Reset all joints to 0 (J0 to J5)
2. Verification of final positions

### Troubleshooting

#### Common Issues

**No camera detected**
```
# Check the device
ls /dev/video*
# Change the index in the code if necessary
self.cap = cv2.VideoCapture(2)  # Change the 2
```

**No frame detected**
- Check the lighting
- Adjust the color thresholds in the parameters
- Verify that the frame is clearly visible

**ROS positions not received**
```
# Check the topics
ros2 topic list
# Check the messages
ros2 topic echo /arm/joint_state
```

**Z not measured**
- Perform a full alignment (key a)
- Use the Z command in a file

#### Error Messages

| Message | Solution |
|---------|----------|
| No frame detected | Check the camera and lighting |
| Z not measured! | Perform an alignment (A) or Z measurement |
| J2 not descended! | Check automatic descent |
| Bar X out of range | Check OFFSET_PINCE or J5 rotation |

### Logs

The program generates detailed logs in the console:

```
[INFO] [BarreDetectionNode]: === DESCENT PHASE (no camera) ===
[INFO] [BarreDetectionNode]: Total distance: 150.0 mm
[INFO] [BarreDetectionNode]:   J2: -100.0 mm (2/3)
[INFO] [BarreDetectionNode]:   J4: -50.0 mm (1/3)
[INFO] [BarreDetectionNode]:  J0 (horizontal): +32.8 mm
[INFO] [BarreDetectionNode]: J3 compensation (vertical): +7.8 mm
```

### Architecture

```
BarreDetectionNode
+-- ROS Interface
|   +-- Publishers (/arm/joint_position, /wrist/joint_position)
|   +-- Subscribers (/arm/joint_state, /wrist/joint_state)
+-- Vision
|   +-- Frame detection (HSV masks)
|   +-- Perspective warp
|   +-- Bar detection (horizontal projection)
+-- Movements
|   +-- Alignments (J0, J1, J3)
|   +-- Calculated descent
|   +-- Navigation
|   +-- Inspection
+-- User Interface
    +-- Keyboard commands
    +-- Command files
```
