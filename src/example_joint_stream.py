from egm import EGM

# Robot Module #
# First execute the following code in RobotStudio and then run the python code.
# MODULE MainModule
#     VAR egmident egmID1;
#     VAR egmstate egmSt1;
#     VAR num count:=0;

#     PROC main()
#         EGMReset egmID1;
#         EGMGetId egmID1;
#         egmSt1:=EGMGetState(egmID1);
#         IF egmSt1<=EGM_STATE_CONNECTED THEN
#             EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Joint;
#         ENDIF
#         EGMActJoint egmID1;
#         EGMActJoint egmID1\StreamStart;
#         WHILE count<10 DO
#             Incr count;
#             MoveAbsJ [[0,0,0,0,30,0],[9E9,9E9,9E9,9E9,9E9,9E9]]\NoEOffs,v1000,fine,tool0\WObj:=wobj0;
#             MoveAbsJ [[90,0,0,0,30,0],[9E9,9E9,9E9,9E9,9E9,9E9]]\NoEOffs,v1000,fine,tool0\WObj:=wobj0;
#         ENDWHILE
#         EGMStop egmID1,EGM_STOP_HOLD;
#     ENDPROC
# ENDMODULE


def main() -> None:
    egm = EGM()

    while True:
        success, state = egm.receive_from_robot()
        if success:
            print(state.joint_angles[0], state.joint_angles[1], state.joint_angles[2],
                  state.joint_angles[3], state.joint_angles[4], state.joint_angles[5])
        else:
            print("Failed to receive from robot")
            break


if __name__ == "__main__":
    main()
