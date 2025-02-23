# ABBRobotEGM

![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

`ABBRobotEGM` is a Python library for interfacing with ABB robots using Externally Guided Motion (EGM). This library provides real-time streaming communication with ABB robots at rates up to 250Hz using UDP.  It's based on the official ABB EGM [documentation](https://github.com/FLo-ABB/ABB-EGM-Python/blob/main/doc/3HAC073318%20AM%20Externally%20Guided%20Motion%20RW7-en.pdf) and examples, and it's designed to be easy to use and to integrate with other systems.

## Prerequisites

- Python 3.x
- ABB RobotWare 7.X (should work with 6.X with few modifications)
- ABB Robot with EGM option (3124-1 Externally Guided Motion)


## Installation 🚀

### Using pip 🐍

To install the library using pip, run the following command:

```bash
pip install ABBRobotEGM
```

### Manual Installation 📦

To use this library in your project, first download the repository and place the `ABBRobotEGM` folder in your project's directory. You can then import the `EGM` class from this library to use it in your project.

## Simple Examples

The repository includes several examples demonstrating different EGM functionalities. Inside each python example file, you can find the relative **RAPID** code that should be running on the robot controller.

### Guidance Mode

#### 1. Joint

example_joint_guidance.py - Makes the first joint oscillate between -45° and +45°:

#### 2. Cartesian
example_pose_guidance.py - Makes the robot move in a circular pattern

### Streaming Mode

#### 1. Joint Streaming
example_joint_stream.py - Streams robot joint positions :
```python	
from ABBRobotEGM import EGM

def main() -> None:
    """
    Example showing how to stream the robot's position.
    Be sure the robot is running before running this script.
    """
    with EGM() as egm:
        while True:
            success, state = egm.receive_from_robot()
            if not success:
                print("Failed to receive from robot")
                break
            print(f"{state.clock[1]}, {state.joint_angles[0]}, {state.joint_angles[1]}, {state.joint_angles[2]}, {state.joint_angles[3]}, {state.joint_angles[4]}, {state.joint_angles[5]}")


if __name__ == "__main__":
    main()

```	


#### 2. Cartesian Streaming
example_pos_stream.py - Streams robot cartesian position
```python	
from ABBRobotEGM import EGM

def main() -> None:
    """
    Example showing how to stream the robot's position.
    Be sure the robot is running before running this script.
    """
    with EGM() as egm:
        while True:
            success, state = egm.receive_from_robot()
            if not success:
                print("Failed to receive from robot")
                break
            print(f"{state.clock[1]}, {state.cartesian.pos.x}, {state.cartesian.pos.y}, {state.cartesian.pos.z}")


if __name__ == "__main__":
    main()
```

### 3. Data Exchange
example_table.py - Demonstrates exchanging data arrays with the robot

## Complex Scenario
Example of a more complex scenario where the robot is scanning a surface giving in real time its tool center point position and correlating with a sensor reading. Rspag and python code available in *"scenario scan"* folder.

https://github.com/user-attachments/assets/03f151de-e098-4255-ac46-7dff42231071

## Features 🚀

- Real-time communication at up to 250Hz
- Joint position control
- Cartesian position control
- Position streaming
- Path corrections
- RAPID data exchange
- External axis support
- Force measurement reading
- Comprehensive robot state feedback


## Contributing 🤝

Contributions are welcome! Please feel free to submit pull requests.
