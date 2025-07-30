from ABBRobotEGM import EGM
import numpy as np

# Robot Module #
# MODULE MainModule
#     VAR egmident egmID1;
#     CONST pose pose0:=[[0,0,0],[1,0,0,0]];
#     PERS dnum DnumInData{40};
#     PERS dnum DnumOutData{40};
#     VAR dnum count:=0;
#
#     PROC main()
#         EGMGetId egmID1;
#         EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Pose;
#         EGMActPose egmID1\StreamStart\Tool:=tool0\WObj:=wobj0\DataToSensor:=DnumOutData,\DataFromSensor:=DnumInData,pose0,EGM_FRAME_WOBJ,pose0,EGM_FRAME_WOBJ;
#         WHILE count<10 DO
#             Incr count;
#             DnumOutData:=[count,count,count,count,count,count,count,count,count,count,
#                           count,count,count,count,count,count,count,count,count,count,
#                           count,count,count,count,count,count,count,count,count,count,
#                           count,count,count,count,count,count,count,count,count,count];
#             waittime 2;
#         ENDWHILE
#         EGMStop egmID1,EGM_STOP_HOLD;
#     ENDPROC
# ENDMODULE


def main() -> None:
    """
    Example showing how to send and receive data to/from the robot using EGM.
    Be sure the robot is running before running this script.
    """
    with EGM() as egm:
        while True:
            success, state = egm.receive_from_robot()
            if not success:
                print("Failed to receive from robot")
                break
            # Receive table from robot
            array_from_robot = state.rapid_from_robot
            print(array_from_robot)
            # Send table to robot
            array_to_robot: np.ndarray = np.array([np.random.randint(0, 100) for _ in range(40)])
            egm.send_to_robot(joint_angles=None, rapid_to_robot=array_to_robot)


if __name__ == "__main__":
    main()
