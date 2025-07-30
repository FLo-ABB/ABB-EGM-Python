from ABBRobotEGM import EGM
import numpy as np

# Robot Module #
# MODULE MainModule
#     VAR egmident egmID1;
#     VAR egmstate egmSt1;
#     CONST pose pose0:=[[0,0,0],[1,0,0,0]];
#     CONST egm_minmax egm_minmax_lin:=[-1E-9,+1E-9];

#     PROC main()
#         VAR robtarget pNow;
#         pNow:=CRobT();
#         ! Before EGM guidance, the robot has to stop on a fine point
#         MoveL pNow,v1000,fine,tool0\WObj:=wobj0;
#         EGMReset egmID1;
#         EGMGetId egmID1;
#         egmSt1:=EGMGetState(egmID1);
#         IF egmSt1<=EGM_STATE_CONNECTED THEN
#             EGMSetupUC ROB_1,egmID1,"default","UCdevice"\pose;
#         ENDIF
#         EGMActPose egmID1\StreamStart\Tool:=tool0\WObj:=wobj0\DOToSensor:=doEGM\DIFromSensor:=diEGM,pose0,EGM_FRAME_WOBJ,pose0,EGM_FRAME_WOBJ\X:=egm_minmax_lin\Y:=egm_minmax_lin\MaxSpeedDeviation:=4000;
#         EGMRunPose egmID1,EGM_STOP_HOLD\NoWaitCond\x\y\RampInTime:=0.05;
#         WaitDI diEGM,high;
#         EGMStop egmID1,EGM_STOP_HOLD;
#     ENDPROC
# ENDMODULE


def main() -> None:
    """
    Example showing how to make the robot move in a circle using EGM.
    The robot will complete 3 circles and then stop.
    Be sure diEGM is low before running this script.
    Be sure the robot is running before running this script.
    """
    with EGM() as egm:

        # Circle parameters
        angle = 0.0
        radius = 50.0        # Circle radius in mm
        center_x = 250.0     # Circle center X coordinate
        center_y = 0.0       # Circle center Y coordinate
        cycles = 0           # Count completed circles
        step = 0.003        # Angular step (smaller = smoother but slower)

        while True:
            success, state = egm.receive_from_robot()
            if not success:
                print("Failed to receive from robot")
                break

            # Get current orientation (maintain throughout motion)
            current_orient = np.array([
                state.cartesian.orient.u0,  # w
                state.cartesian.orient.u1,  # x
                state.cartesian.orient.u2,  # y
                state.cartesian.orient.u3   # z
            ])

            # Calculate next position on circle
            new_pos = np.array([
                center_x + radius * np.cos(angle),  # X coordinate
                center_y + radius * np.sin(angle),  # Y coordinate
                state.cartesian.pos.z               # Keep current Z
            ])

            # Update angle and count cycles
            angle += step
            if angle >= 2 * np.pi:
                angle = 0
                cycles += 1
                print(f"Completed circle {cycles}")

            # Send position to robot, stop after 3 circles
            egm.send_to_robot(
                cartesian=(new_pos, current_orient),
                digital_signal_to_robot=(cycles >= 3)
            )


if __name__ == "__main__":
    main()
