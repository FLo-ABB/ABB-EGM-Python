from egm import EGM
import numpy as np

# Robot Module #
# MODULE MainModule
#      VAR egmident egmID1;
#      VAR egmstate egmSt1;
#      CONST pose pose0:=[[0,0,0],[1,0,0,0]];
#      CONST egm_minmax egm_minmax_lin:=[-1E-9,+1E-9];

#      PROC main()
#          EGMReset egmID1;
#          EGMGetId egmID1;
#          egmSt1:=EGMGetState(egmID1);
#          IF egmSt1<=EGM_STATE_CONNECTED THEN
#              EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Pose;
#          ENDIF
#          EGMActPose egmID1\StreamStart\Tool:=tool0\WObj:=wobj0,pose0,EGM_FRAME_WOBJ,pose0,EGM_FRAME_WOBJ\X:=egm_minmax_lin;
#          EGMRunPose egmID1, EGM_STOP_HOLD\NoWaitCond \x \RampInTime:=0.05;
#          waittime 10;
#          EGMStop egmID1,EGM_STOP_HOLD;
#      ENDPROC
#  ENDMODULE


def main() -> None:
    egm = EGM()

    while True:
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

            # Move 1mm in x direction
            new_pos = current_pos.copy()
            new_pos[0] += 100.0  # x + 1mm

            # Send new position while maintaining current orientation
            egm.send_to_robot(cartesian=(new_pos, current_orient))
        else:
            print("Failed to receive from robot")
            break


if __name__ == "__main__":
    main()
