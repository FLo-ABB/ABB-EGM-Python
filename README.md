# ABB-EGM-Python

A Python library for interfacing with ABB robots using Externally Guided Motion (EGM). This library provides real-time streaming communication with ABB robots at rates up to 250Hz using UDP.

## Prerequisites

- Python 3.x
- ABB RobotWare 7.X (should work with 6.X with few modifications)
- ABB Robot with EGM option (3124-1 Externally Guided Motion)
- The following Python packages:
  ```txt
  numpy
  protobuf
  ```

## Documentation
Please refer to the [documentation](ttps://github.com/FLo-ABB/ABB-EGM-Python/blob/main/doc/3HAC073318%20AM%20Externally%20Guided%20Motion%20RW7-en.pdfh) for detailed information on EGM option.

## Installation

1. Clone the repository

2. Install dependencies:
```sh
pip install -r requirements.txt
```

## Simple Examples

The library includes several examples demonstrating different EGM functionalities. Inside each python example file, you can find the relative **RAPID** code that should be running on the robot controller.

### Guidance Mode

#### 1. Joint

example_joint_guidance.py - Makes the first joint oscillate between -45° and +45°:

#### 2. Cartesian
example_pose_guidance.py - Makes the robot move in a circular pattern

### Streaming Mode

#### 1. Joint Streaming
example_joint_stream.py - Streams robot joint positions

#### 2. Cartesian Streaming
example_pos_stream.py - Streams robot cartesian position

### 3. Data Exchange
example_table.py - Demonstrates exchanging data arrays with the robot

## Complex Scenario
Example of a more complex scenario where the robot is scanning a surface giving in real time its tool center point position and correlating with a sensor reading. Rspag and python code available in *"scenario scan"* folder.

https://github.com/user-attachments/assets/03f151de-e098-4255-ac46-7dff42231071

## Features

- Real-time communication at up to 250Hz
- Joint position control
- Cartesian position control
- Position streaming
- Path corrections
- RAPID data exchange
- External axis support
- Force measurement reading
- Comprehensive robot state feedback

## License

Licensed under the Apache License, Version 2.0. See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit pull requests.
