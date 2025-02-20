from egm import EGM

# Robot Module :

# MODULE MainModule
#     VAR egmident egmID1;
#     VAR egmstate egmSt1;

#     PROC main()
#         EGMReset egmID1;
#         EGMGetId egmID1;
#         egmSt1:=EGMGetState(egmID1);
#         IF egmSt1<=EGM_STATE_CONNECTED THEN
#             EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Joint;
#         ENDIF
#         EGMActJoint egmID1;
#         EGMRunJoint egmID1, EGM_STOP_HOLD\NoWaitCond \J1 \RampInTime:=0.05;
#         waittime 10;
#         EGMStop egmID1,EGM_STOP_HOLD;
#     ENDPROC
# ENDMODULE


def main() -> None:
    egm = EGM()

    while True:
        success, state = egm.receive_from_robot()
        if success:
            # Get the actual joint angles and modify the first joint angle by 1 degree
            joint_angles = state.joint_angles.copy()
            joint_angles[0] -= 1

            # Update the joint angles in the state
            new_state = state._replace(joint_angles=joint_angles)

            # Send it to the robot
            egm.send_to_robot(new_state.joint_angles)
        else:
            print("Failed to receive from robot")
            break


if __name__ == "__main__":
    main()
