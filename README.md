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

#### 3. Speed Control
example_speed_guidance.py - Demonstrates how to control the robot movement with speed references.

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

### 3. Path Correction
example_path_correction.py - Apply dynamic path corrections during robot movement:
```python
from ABBRobotEGM import EGM
import numpy as np
import time

def main() -> None:
    """
    Example showing how to apply path corrections during robot movement.
    This will apply a sinusoidal correction in the Y direction while the robot
    moves along a straight line in the X direction (using EGMMoveL in RAPID).
    
    The sinusoidal pattern:
    - Amplitude: 5mm
    - Frequency: 0.7 Hz
    """
    with EGM() as egm:
        print("Waiting for initial message from robot...")
        while True:
            success, _ = egm.receive_from_robot(timeout=1.0)
            if success:
                print("Connected to robot!")
                break
            
        # Parameters for sinusoidal correction
        amplitude = 5.0    # mm
        frequency = 0.7      # Hz
        t_start = time.time()
        
        print("Sending Y-axis path corrections...")
        while True:
            success, _ = egm.receive_from_robot(timeout=0.1)
            if not success:
                print("Lost connection to robot")
                break
                
            # Calculate Y correction using sine wave
            t = time.time() - t_start
            y_correction = amplitude * np.sin(2 * np.pi * frequency * t)
            
            # Create correction vector [x, y, z]
            correction = np.array([0.0, y_correction, 0.0])
            
            # Send path correction
            egm.send_to_robot_path_corr(correction, age=1)
            
            # Match robot's sensor refresh rate of 48ms
            time.sleep(0.048)

if __name__ == "__main__":
    main()
```

RAPID code that should be running on the robot:
```rapid
MODULE MainModule
    VAR egmident egmID1;
    
    PROC main()
        EGMReset egmID1;
        EGMGetId egmID1;
        EGMSetupUC ROB_1,egmId1,"EGMPathCorr","UCdevice"\PathCorr\APTR;
        EGMActMove EGMid1,MyTool.tframe\SampleRate:=48;
        MoveL StartPos,vmax,fine,MyTool\WObj:=WObj0;
        ! Move in X direction while Python applies Y corrections
        EGMMoveL egmID1,Offs(StartPos,400,0,0),v100,fine,MyTool\WObj:=WObj0;
        MoveL StartPos,vmax,fine,MyTool\WObj:=WObj0;
        EGMStop egmID1,EGM_STOP_HOLD;
    ENDPROC
ENDMODULE
```

### 4. Data Exchange
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
