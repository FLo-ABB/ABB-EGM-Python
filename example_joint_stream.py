from abb_egm.egm import EGM

# Robot Module #
# MODULE MainModule
#     VAR egmident egmID1;
#     VAR num count:=0;

#     PROC main()
#         EGMReset egmID1;
#         EGMGetId egmID1;
#         EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Joint;
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
