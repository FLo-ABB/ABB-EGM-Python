from egm import EGM

# Robot Module #
# First execute the following code in RobotStudio and then run the python code.
# MODULE Module1
#     VAR egmident egmID1;
#     VAR num count:=0;
#     CONST pose pose0:=[[0,0,0],[1,0,0,0]];
#     PROC main()
#         ! EGM Datastream
#         EGMGetId egmID1;
#         EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Pose;
#         EGMActPose egmID1\StreamStart\Tool:=tool0\WObj:=wobj0\DataToSensor:=DnumOutData,\DataFromSensor:=DnumInData,pose0,EGM_FRAME_WOBJ,pose0,EGM_FRAME_WOBJ;
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
            print(state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z)
        else:
            print("Failed to receive from robot")
            break


if __name__ == "__main__":
    main()
