from abb_egm.egm import EGM

# MODULE MainModule
#     VAR egmident egmID1;
#     VAR num count:=0;
#     CONST pose pose0:=[[0,0,0],[1,0,0,0]];

#     PROC main()
#         ! EGM Datastream
#         EGMGetId egmID1;
#         EGMSetupUC ROB_1,egmID1,"default","UCdevice"\pose;
#         EGMActPose egmID1\StreamStart\Tool:=tool0\WObj:=wobj0,pose0,EGM_FRAME_WOBJ,pose0,EGM_FRAME_WOBJ;
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
            print(f"{state.clock[1]}, {state.cartesian.pos.x}, {state.cartesian.pos.y}, {state.cartesian.pos.z}")


if __name__ == "__main__":
    main()
