# Copyright 2022 Wason Technology LLC, Rensselaer Polytechnic Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket
import select
import numpy as np
import errno
from typing import Tuple, NamedTuple, Any, Optional
from ABBRobotEGM._egm_protobuf import egm_pb2
from .abb_data import Pos, Orientation, Euler, Pose

# Constants
DEFAULT_PORT = 6510
BUFFER_SIZE = 65536
TIMEOUT = 0.5


class EGMHeader(NamedTuple):
    """Represents EGM message header"""
    seqno: int
    tm: int
    mtype: str


class EGMTimeStamp(NamedTuple):
    """Represents PTP/IEEE-1588 timestamp."""
    seconds: int
    nanoseconds: int


class EGMClock(NamedTuple):
    """Represents Unix timestamp."""
    seconds: int
    microseconds: int


class EGMRobotState(NamedTuple):
    """Represents the state of the robot as received from EGM feedback."""
    # Header information
    header: EGMHeader

    # Feedback state
    joint_angles: np.array
    cartesian: Optional[Pose]
    external_axes: np.array
    clock: Optional[Tuple[int, int]]       # EgmClock (seconds, microseconds)
    timestamp: Optional[Tuple[int, int]]   # EgmTimestamp (seconds, nanoseconds)

    # Planned state
    joint_angles_planned: np.array
    cartesian_planned: Optional[Pose]
    external_axes_planned: np.array
    planned_clock: Optional[Tuple[int, int]]
    planned_timestamp: Optional[Tuple[int, int]]

    # Motor and execution state
    motors_on: bool
    mci_state: str
    mci_convergence_met: bool
    rapid_running: bool

    # Forces and motion
    measured_force: np.array
    utilization_rate: float
    move_index: int

    # Collision and diagnostics
    collision_info: Any
    test_signals: np.array

    # RAPID interface
    rapid_from_robot: np.array

    # Additional data
    torque_ref: np.array
    robot_message: Any  # Raw message for debugging


class EGM:
    """
    ABB EGM (Externally Guided Motion) client. EGM provides a real-time streaming connection to the robot using
    UDP, typically at a rate of 250 Hz. The robot controller initiates the connection. The IP address and port of the
    client must be configured on the robot controller side. The EGM client will send commands to the port it receives
    packets from.

    :param port: The port to receive UDP packets. Defaults to 6510
    """

    def __init__(self, port: int = DEFAULT_PORT):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', port))
        self.send_sequence_number = 0
        self.egm_addr = None
        self.count = 0

    def __enter__(self):
        """Enter the context manager protocol."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit the context manager protocol."""
        self.close()

    def receive_from_robot(self, timeout: float = TIMEOUT) -> Tuple[bool, Optional[EGMRobotState]]:
        """
        Receive feedback from the robot. Specify an optional timeout. Returns a tuple with success and the current
        robot state.

        :param timeout: Timeout in seconds. May be zero to immediately return if there is no new data.
        :return: Success and robot state as a tuple
        """
        try:
            res = select.select([self.socket], [], [self.socket], timeout)
        except select.error as err:
            if err.args[0] == errno.EINTR:
                return False, None
            raise

        if not res[0] and not res[2]:
            return False, None

        try:
            buf, addr = self.socket.recvfrom(BUFFER_SIZE)
        except socket.error:
            self.egm_addr = None
            return False, None

        self.egm_addr = addr
        robot_message = egm_pb2.EgmRobot()
        robot_message.ParseFromString(buf)

        state = self._parse_robot_message(robot_message)
        return True, state

    def _parse_robot_message(self, robot_message: Any) -> EGMRobotState:
        """Parse the robot message and return the robot state."""
        # Get header information
        header = EGMHeader(
            seqno=robot_message.header.seqno if robot_message.HasField('header') else 0,
            tm=robot_message.header.tm if robot_message.HasField('header') else 0,
            mtype=robot_message.header.mtype if robot_message.HasField('header') else "UNDEFINED"
        )

        # Get feedback data
        joint_angles = self._get_joint_angles(robot_message)
        cartesian = self._get_cartesian(robot_message)
        external_axes = self._get_external_axes(robot_message)
        clock, timestamp = self._get_timestamps(robot_message)

        # Get planned data
        joint_angles_planned = self._get_joint_angles_planned(robot_message)
        cartesian_planned = self._get_cartesian_planned(robot_message)
        external_axes_planned = self._get_external_axes_planned(robot_message)
        planned_clock, planned_timestamp = self._get_planned_timestamps(robot_message)

        # Get motor and execution state
        motors_on = self._get_motors_on(robot_message)
        mci_state = self._get_mci_state(robot_message)
        mci_convergence_met = self._get_mci_convergence_met(robot_message)
        rapid_running = self._get_rapid_running(robot_message)

        # Get forces and motion data
        measured_force = self._get_measured_force(robot_message)
        utilization_rate = self._get_utilization_rate(robot_message)
        move_index = self._get_move_index(robot_message)

        # Get collision and diagnostics
        collision_info = self._get_collision_info(robot_message)
        test_signals = self._get_test_signals(robot_message)

        # Get RAPID and torque data
        rapid_from_robot = self._get_rapid_from_robot(robot_message)
        torque_ref = self._get_torque_ref(robot_message)

        return EGMRobotState(
            header=header,
            joint_angles=joint_angles,
            cartesian=cartesian,
            external_axes=external_axes,
            clock=clock,
            timestamp=timestamp,
            joint_angles_planned=joint_angles_planned,
            cartesian_planned=cartesian_planned,
            external_axes_planned=external_axes_planned,
            planned_clock=planned_clock,
            planned_timestamp=planned_timestamp,
            motors_on=motors_on,
            mci_state=mci_state,
            mci_convergence_met=mci_convergence_met,
            rapid_running=rapid_running,
            measured_force=measured_force,
            utilization_rate=utilization_rate,
            move_index=move_index,
            collision_info=collision_info,
            test_signals=test_signals,
            rapid_from_robot=rapid_from_robot,
            torque_ref=torque_ref,
            robot_message=robot_message
        )

    def _get_joint_angles(self, robot_message: Any) -> np.array:
        if robot_message.HasField('feedBack'):
            return np.array(list(robot_message.feedBack.joints.joints))
        return np.array([])

    def _get_rapid_running(self, robot_message: Any) -> bool:
        if robot_message.HasField('rapidExecState'):
            return robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
        return False

    def _get_motors_on(self, robot_message: Any) -> bool:
        if robot_message.HasField('motorState'):
            return robot_message.motorState.state == robot_message.motorState.MOTORS_ON
        return False

    def _get_cartesian(self, robot_message: Any) -> Optional[Pose]:
        if robot_message.HasField('feedBack') and robot_message.feedBack.HasField('cartesian'):
            cart_p = robot_message.feedBack.cartesian.pos
            cart_q = robot_message.feedBack.cartesian.orient
            return Pose(
                pos=Pos(cart_p.x, cart_p.y, cart_p.z),
                orient=Orientation(cart_q.u0, cart_q.u1, cart_q.u2, cart_q.u3),
                euler=Euler(0, 0, 0)  # Assuming Euler angles are not provided in the message
            )
        return None

    def _get_external_axes(self, robot_message: Any) -> np.array:
        if robot_message.HasField('feedBack') and robot_message.feedBack.HasField('externalJoints'):
            return np.array(list(robot_message.feedBack.externalJoints.joints))
        return np.array([])

    def _get_external_axes_planned(self, robot_message: Any) -> np.array:
        if robot_message.HasField('planned') and robot_message.planned.HasField('externalJoints'):
            return np.array(list(robot_message.planned.externalJoints.joints))
        return np.array([])

    def _get_joint_angles_planned(self, robot_message: Any) -> np.array:
        if robot_message.HasField('planned') and robot_message.planned.HasField('joints'):
            return np.array(list(robot_message.planned.joints.joints))
        return np.array([])

    def _get_cartesian_planned(self, robot_message: Any) -> Optional[Pose]:
        if robot_message.HasField('planned') and robot_message.planned.HasField('cartesian'):
            cart_p = robot_message.planned.cartesian.pos
            cart_q = robot_message.planned.cartesian.orient
            return Pose(
                pos=Pos(cart_p.x, cart_p.y, cart_p.z),
                orient=Orientation(cart_q.u0, cart_q.u1, cart_q.u2, cart_q.u3),
                euler=Euler(0, 0, 0)  # Assuming Euler angles are not provided in the message
            )
        return None

    def _get_measured_force(self, robot_message: Any) -> np.array:
        if robot_message.HasField('measuredForce'):
            force_active = robot_message.measuredForce.fcActive if robot_message.measuredForce.HasField('fcActive') else True
            if force_active:
                return np.array(list(robot_message.measuredForce.force))
        return np.array([])

    def _get_move_index(self, robot_message: Any) -> Optional[int]:
        if robot_message.HasField('moveIndex'):
            return robot_message.moveIndex
        return None

    def _get_rapid_from_robot(self, robot_message: Any) -> np.array:
        if robot_message.HasField('RAPIDfromRobot'):
            return np.array(list(robot_message.RAPIDfromRobot.dnum))
        return np.array([])

    def _get_mci_state(self, robot_message: Any) -> Optional[str]:
        if robot_message.HasField('mciState'):
            return robot_message.mciState.state
        return None

    def _get_mci_convergence_met(self, robot_message: Any) -> Optional[bool]:
        if robot_message.HasField('mciConvergenceMet'):
            return robot_message.mciConvergenceMet
        return None

    def _get_test_signals(self, robot_message: Any) -> np.array:
        if robot_message.HasField('testSignals'):
            return np.array(list(robot_message.testSignals.signals))
        return np.array([])

    def _get_utilization_rate(self, robot_message: Any) -> Optional[float]:
        if robot_message.HasField('utilizationRate'):
            return robot_message.utilizationRate
        return None

    def _get_collision_info(self, robot_message: Any) -> Optional[Any]:
        if robot_message.HasField('CollisionInfo'):
            return robot_message.CollisionInfo
        return None

    def debug_print_robot_message(self, robot_message: Any):
        """
        Print the robot message for debugging purposes.

        :param robot_message: The robot message to print
        """
        print(robot_message)

    def send_to_robot(self, joint_angles: Optional[np.array] = None,
                      cartesian: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                      speed_ref: np.array = None, external_joints: np.array = None,
                      external_joints_speed: np.array = None, rapid_to_robot: np.array = None, digital_signal_to_robot: bool = None) -> bool:
        """
        Send a command to robot. Returns False if no data has been received from the robot yet.
        Either joint_angles or cartesian must be provided.

        :param joint_angles: Joint angle command in degrees
        :param cartesian: Tuple of (position, orientation) where:
                        - position is [x,y,z] in millimeters
                        - orientation is quaternion [w,x,y,z]
        :param speed_ref: Speed reference for the joints
        :param external_joints: External joints positions
        :param external_joints_speed: External joints speed reference
        :param rapid_to_robot: RAPID data to send to robot
        :return: True if successful, False if no data received from robot yet
        """
        if not self.egm_addr:
            return False

        # if joint_angles is None and cartesian is None:
        #     raise ValueError("Either joint_angles or cartesian must be provided")

        self.send_sequence_number += 1

        if cartesian is not None:
            # If cartesian is provided, use cartesian control
            pos, orient = cartesian
            sensor_message = self._create_sensor_message_cart(
                pos, orient, speed_ref, external_joints, external_joints_speed, rapid_to_robot, digital_signal_to_robot
            )
        else:
            # Use joint control
            sensor_message = self._create_sensor_message(
                joint_angles, speed_ref, external_joints, external_joints_speed, rapid_to_robot, digital_signal_to_robot
            )

        return self._send_message(sensor_message)

    def _create_sensor_message(self, joint_angles: np.array, speed_ref: np.array, external_joints: np.array,
                               external_joints_speed: np.array, rapid_to_robot: np.array, digital_signal_to_robot: bool) -> Any:
        """Create the sensor message to be sent to the robot."""
        sensor_message = egm_pb2.EgmSensor()
        header = sensor_message.header
        header.mtype = egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno = self.send_sequence_number
        self.send_sequence_number += 1

        planned = sensor_message.planned
        speed_ref_message = sensor_message.speedRef

        if joint_angles is not None:
            joint_angles = self._flatten_and_convert_to_list(joint_angles)
            planned.joints.joints.extend(joint_angles)

        if speed_ref is not None:
            speed_ref = self._flatten_and_convert_to_list(speed_ref)
            speed_ref_message.joints.joints.extend(speed_ref)

        if external_joints is not None:
            external_joints = self._flatten_and_convert_to_list(external_joints)
            planned.externalJoints.joints.extend(external_joints)

        if external_joints_speed is not None:
            external_joints_speed = self._flatten_and_convert_to_list(external_joints_speed)
            speed_ref_message.externalJoints.joints.extend(external_joints_speed)

        if rapid_to_robot is not None:
            rapid_to_robot = self._flatten_and_convert_to_list(rapid_to_robot)
            sensor_message.RAPIDtoRobot.dnum.extend(rapid_to_robot)

        if digital_signal_to_robot is not None:
            sensor_message.RAPIDtoRobot.digVal = digital_signal_to_robot

        return sensor_message

    def _flatten_and_convert_to_list(self, array: np.array) -> list:
        """Flatten the array and convert it to a list, ensuring homogeneous shape."""
        try:
            array = np.asarray(array, dtype=np.float64).flatten()
        except ValueError:
            raise ValueError("Input array has an inhomogeneous shape or contains non-numeric elements.")
        return array.tolist()

    def _send_message(self, message: Any) -> bool:
        """Send the serialized message to the robot."""
        buf = message.SerializeToString()
        try:
            self.socket.sendto(buf, self.egm_addr)
        except socket.error:
            return False
        return True

    def send_to_robot_cart(self, pos: np.ndarray, orient: np.ndarray, speed_ref: np.array = None,
                           external_joints: np.array = None, external_joints_speed: np.array = None,
                           rapid_to_robot: np.array = None, digital_signal_to_robot: bool = None) -> bool:
        """
        Send a cartesian command to robot. Returns False if no data has been received from the robot yet. The pose
        is relative to the tool, workobject, and frame specified when the EGM operation is initialized. The EGM
        operation must have been started with EGMActPose and EGMRunPose.

        :param pos: The position of the TCP in millimeters [x,y,z]
        :param orient: The orientation of the TCP in quaternions [w,x,y,z]
        :return: True if successful, False if no data received from robot yet
        """
        if not self.egm_addr:
            return False

        self.send_sequence_number += 1
        sensor_message = self._create_sensor_message_cart(pos, orient, speed_ref, external_joints,
                                                          external_joints_speed, rapid_to_robot, digital_signal_to_robot)
        return self._send_message(sensor_message)

    def _create_sensor_message_cart(self, pos: np.ndarray, orient: np.ndarray, speed_ref: np.array,
                                    external_joints: np.array, external_joints_speed: np.array,
                                    rapid_to_robot: np.array, digital_signal_to_robot: bool) -> Any:
        """Create the sensor message with cartesian data to be sent to the robot."""
        sensor_message = egm_pb2.EgmSensor()
        header = sensor_message.header
        header.mtype = egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
        header.seqno = self.send_sequence_number
        self.send_sequence_number += 1

        planned = sensor_message.planned
        speed_ref_message = sensor_message.speedRef

        if pos is not None and orient is not None:
            planned.cartesian.pos.x = pos[0]
            planned.cartesian.pos.y = pos[1]
            planned.cartesian.pos.z = pos[2]
            planned.cartesian.orient.u0 = orient[0]
            planned.cartesian.orient.u1 = orient[1]
            planned.cartesian.orient.u2 = orient[2]
            planned.cartesian.orient.u3 = orient[3]

        if speed_ref is not None:
            speed_ref_message.cartesians.value.extend(list(np.array(speed_ref)))

        if external_joints is not None:
            planned.externalJoints.joints.extend(list(np.array(external_joints)))

        if external_joints_speed is not None:
            speed_ref_message.externalJoints.joints.extend(list(np.array(external_joints_speed)))

        if rapid_to_robot is not None:
            sensor_message.RAPIDtoRobot.dnum.extend(list(np.array(rapid_to_robot)))

        if digital_signal_to_robot is not None:
            sensor_message.RAPIDtoRobot.digVal = digital_signal_to_robot

        return sensor_message

    def send_to_robot_path_corr(self, pos: np.ndarray, age: float = 1) -> bool:
        """
        Send a path correction command. Returns False if no data has been received from the robot yet. The path
        correction is a displacement [x,y,z] in millimeters in **path coordinates**. The displacement uses
        "path coordinates", which relate the direction of movement of the end effector. See `CorrConn` command in
        *Technical reference manual RAPID Instructions, Functions and Data types* for a detailed description of path
        coordinates.  The EGM operation must have been started with EGMActMove, and use EGMMoveL and EGMMoveC commands.

        :param pos: The displacement in path coordinates in millimeters [x,y,z]
        :return: True if successful, False if no data received from robot yet
        """
        self.send_sequence_number += 1
        sensor_message = self._create_sensor_message_path_corr(pos, age)
        return self._send_message(sensor_message)

    def _create_sensor_message_path_corr(self, pos: np.ndarray, age: float) -> Any:
        """Create the sensor message with path correction data to be sent to the robot."""
        sensor_message = egm_pb2.EgmSensorPathCorr()
        header = sensor_message.header
        header.mtype = egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_PATH_CORRECTION')
        header.seqno = self.send_sequence_number
        self.send_sequence_number += 1

        path_corr = sensor_message.pathCorr
        path_corr.pos.x = pos[0]
        path_corr.pos.y = pos[1]
        path_corr.pos.z = pos[2]
        path_corr.age = age

        return sensor_message

    def _get_timestamps(self, robot_message: Any) -> Tuple[Optional[EGMClock], Optional[EGMTimeStamp]]:
        clock = None
        timestamp = None

        if robot_message.HasField('feedBack'):
            fb = robot_message.feedBack
            if fb.HasField('time'):
                clock = EGMClock(fb.time.sec, fb.time.usec)
            if fb.HasField('timeStamp'):
                timestamp = EGMTimeStamp(fb.timeStamp.sec, fb.timeStamp.nsec)

        return clock, timestamp

    def _get_planned_timestamps(self, robot_message: Any) -> Tuple[Optional[Tuple[int, int]], Optional[Tuple[int, int]]]:
        """Get time and timestamp from planned data."""
        clock = None
        timestamp = None

        if robot_message.HasField('planned'):
            if robot_message.planned.HasField('time'):
                clock = (robot_message.planned.time.sec,
                         robot_message.planned.time.usec)
            if robot_message.planned.HasField('timeStamp'):
                timestamp = (robot_message.planned.timeStamp.sec,
                             robot_message.planned.timeStamp.nsec)

        return clock, timestamp

    def _get_torque_ref(self, robot_message: Any) -> np.array:
        """Get torque reference values."""
        if robot_message.HasField('torqueRef'):
            return np.array(list(robot_message.torqueRef.joints))
        return np.array([])

    def close(self):
        """Close the connection to the robot."""
        try:
            self.socket.close()
            self.socket = None
        except socket.error:
            pass
