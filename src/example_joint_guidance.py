from egm import EGM

# Robot Module #
# First execute the following code in RobotStudio and then run the python code before the waittime ends.
# The robot will move the first joint by 1 degree until the end of the waittime.
#  MODULE MainModule
#      VAR egmident egmID1;

#      PROC main()
#          VAR jointtarget jNow;
#          jNow := CJointT();
#          ! Before EGM guidance, the robot has to stop on a fine point
#          MoveAbsJ jNow,v1000,fine,tool0\WObj:=wobj0;
#          EGMReset egmID1;
#          EGMGetId egmID1;
#          EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Joint;
#          EGMActJoint egmID1\MaxPosDeviation:=360\MaxSpeedDeviation:=1000;
#          EGMRunJoint egmID1, EGM_STOP_HOLD\NoWaitCond \J1 \RampInTime:=0.05;
#          waittime 10;
#          EGMStop egmID1,EGM_STOP_HOLD;
#      ENDPROC
#  ENDMODULE


def main() -> None:
    egm = EGM()

    while True:
        success, state = egm.receive_from_robot()
        if success:
            # Get the actual joint angles and modify the first joint angle by 1 degree
            joint_angles = state.joint_angles.copy()
            joint_angles[0] += 10

            # Update the joint angles in the state
            new_state = state._replace(joint_angles=joint_angles)

            # Send it to the robot
            egm.send_to_robot(new_state.joint_angles)
        else:
            print("Failed to receive from robot")
            break


if __name__ == "__main__":
    main()
