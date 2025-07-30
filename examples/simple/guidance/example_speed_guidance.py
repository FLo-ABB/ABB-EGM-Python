from ABBRobotEGM import EGM
import numpy as np
import time

# # Robot Module #
# MODULE MainModule
#      VAR egmident egmID1;
#      VAR egmstate egmSt1;
#      CONST pose pose0:=[[0,0,0],[1,0,0,0]];
#      CONST egm_minmax egm_minmax_lin:=[-1E-9,+1E-9];
#      CONST egm_minmax egm_minmax_rot:=[-1E-9,+1E-9];

#      PROC main()
#          Path_20;
#          EGMReset egmID1;
#          EGMGetId egmID1;
#          egmSt1:=EGMGetState(egmID1);
#          IF egmSt1<=EGM_STATE_CONNECTED THEN
#              EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Pose;
#          ENDIF
#          EGMActPose egmID1\StreamStart\Tool:=tool0\WObj:=wobj0,pose0,EGM_FRAME_WOBJ,pose0,EGM_FRAME_WOBJ\z:=egm_minmax_lin\Rz:=egm_minmax_rot\MaxSpeedDeviation:=300;
#          EGMRunPose egmID1, EGM_STOP_HOLD\NoWaitCond \z\Rz;
#          waittime 10;
#          EGMStop egmID1,EGM_STOP_HOLD;
#      ENDPROC
# ENDMODULE


def calculate_lin_span_speed(positions, timestamps):
    """
    Calculate speed using span method (oldest to newest sample).

    :param positions: List of position values
    :param timestamps: List of corresponding timestamps
    :return: Speed in units/second, or 0.0 if insufficient data
    """
    if len(positions) < 2 or len(timestamps) < 2:
        return 0.0

    time_span = timestamps[-1] - timestamps[0]
    position_span = positions[-1] - positions[0]

    return position_span / time_span if time_span > 0 else 0.0


def calculate_angular_span_speed(orientations, timestamps):
    """
    Calculate angular speed using span method (oldest to newest sample).

    :param orientations: List of orientation values (quaternions as numpy arrays)
    :param timestamps: List of corresponding timestamps
    :return: Angular speed in degrees/second, or 0.0 if insufficient data
    """
    if len(orientations) < 2 or len(timestamps) < 2:
        return 0.0

    time_span = timestamps[-1] - timestamps[0]
    if time_span <= 0:
        return 0.0

    # Calculate quaternion difference properly
    q1 = orientations[0]  # Initial quaternion [w, x, y, z]
    q2 = orientations[-1]  # Final quaternion [w, x, y, z]

    # Calculate relative rotation using quaternion conjugate
    # q_rel = q2 * conjugate(q1)
    q1_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])  # Conjugate of q1

    # Quaternion multiplication: q2 * q1_conj (we only need the w component)
    w = q2[0] * q1_conj[0] - q2[1] * q1_conj[1] - q2[2] * q1_conj[2] - q2[3] * q1_conj[3]

    # Convert to rotation angle (in radians)
    # For general case: angle = 2 * arccos(|w|)
    angle_rad = 2 * np.arccos(np.clip(abs(w), 0, 1))

    # Convert to degrees and divide by time span
    angular_speed = np.degrees(angle_rad) / time_span
    return angular_speed


def main() -> None:
    """Simple speed guidance example with span-based speed calculation."""
    egm = EGM()

    start_time = time.time()
    duration = 5.0

    # Variables for span-based speed calculation
    position_history = []
    orientation_history = []
    time_history = []
    max_samples = 8  # Number of samples for averaging
    last_print_time = 0.0

    while time.time() - start_time < duration:
        success, state = egm.receive_from_robot()
        if success:
            # Get current position and orientation
            current_pos = np.array([
                state.cartesian.pos.x,
                state.cartesian.pos.y,
                state.cartesian.pos.z
            ])
            current_orient = np.array([
                state.cartesian.orient.u0,  # w
                state.cartesian.orient.u1,  # x
                state.cartesian.orient.u2,  # y
                state.cartesian.orient.u3   # z
            ])

            # Create a combined speed reference
            speed_ref = egm.create_combined_speed_ref(vz=50, wz=5)  # 50 mm/s upward, 5 deg/s rotation

            # Send current position with speed reference
            egm.send_to_robot_cart(current_pos, current_orient, speed_ref)

            # Calculate and print progress
            current_time = time.time()
            elapsed = current_time - start_time

            # Store current position and time for span-based calculation
            position_history.append(current_pos[2])  # Store Z position
            orientation_history.append(current_orient.copy())  # Store full quaternion
            time_history.append(current_time)

            # Keep only the last max_samples entries
            if len(position_history) > max_samples:
                position_history.pop(0)
                orientation_history.pop(0)
                time_history.pop(0)

            # Calculate span-based speed using helper function
            lin_span_speed = calculate_lin_span_speed(position_history, time_history)
            angular_span_speed = calculate_angular_span_speed(orientation_history, time_history)

            # Print every 0.2 seconds
            if elapsed - last_print_time >= 0.2:
                print(f"Time: {elapsed:.1f}s, Z pos: {current_pos[2]:.1f}mm, "
                      f"Speed: {lin_span_speed:.1f}mm/s, Angular Speed: {angular_span_speed:.1f}deg/s")
                last_print_time = elapsed

        else:
            print("Failed to receive from robot")
            break

    print("Speed guidance example completed!")


if __name__ == "__main__":
    main()
