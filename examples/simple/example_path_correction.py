from ABBRobotEGM import EGM
import numpy as np
import time

# Ensure the robot is configured correctly by setting up "EGMPathCorr" with "Path" level in the Motion configuration.
# This can be done in the robot's controller under: Motion configuration -> External Motion Interface Data.
# MODULE MainModule
# VAR egmident egmID1;
# CONST robtarget Target_10:=[[0.771953305,2.548198209,204.864360938],[-0.000000176,-0.000000006,1,-0.000000002],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
# TASK PERS wobjdata Workobject_1:=[FALSE,TRUE,"",[[525,-125,308],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
# PERS tooldata Pen_TCP:=[TRUE,[[110,0,140],[0.707106781,0,0.707106781,0]],[1,[10.7435139,0,140],[1,0,0,0],0,0,0]];

# PROC main()
#     EGMReset egmID1;
#     EGMGetId egmID1;
#     EGMSetupUC ROB_1,egmId1,"EGMPathCorr","UCdevice"\PathCorr\APTR;
#     EGMActMove EGMid1,Pen_TCP.tframe\SampleRate:=48;
#     MoveL Target_10,vmax,fine,Pen_TCP\WObj:=Workobject_1;
#     EGMMoveL egmID1,Offs(Target_10,200,0,0),v10,fine,Pen_TCP\WObj:=Workobject_1;
#     MoveL Offs(Target_10,0,0,200),vmax,fine,Pen_TCP\WObj:=Workobject_1;
#     EGMStop egmID1,EGM_STOP_HOLD;
# ENDPROC
# ENDMODULE


def main() -> None:
    """
    Example showing how to apply path corrections during robot movement.
    This will apply a sinusoidal correction in the Y direction while the robot
    moves along a straight line in the X direction (using EGMMoveL in RAPID).

    The sinusoidal pattern:
    - Amplitude: 5mm
    - Frequency: 0.7 Hz

    This creates a wavy pattern perpendicular to the robot's movement direction. Run the python script before
    running the RAPID program on the robot.
    """
    with EGM() as egm:
        print("Waiting for initial message from robot...")
        # Wait for first message from robot to establish connection
        while True:
            success, _ = egm.receive_from_robot(timeout=1.0)
            if success:
                print("Connected to robot!")
                break
        # Parameters for sinusoidal correction
        amplitude = 5.0    # mm
        frequency = 0.7      # Hz
        t_start = time.time()
        print("Sending Y-axis path corrections...")

        while True:
            # Always receive from robot first to maintain connection
            success, _ = egm.receive_from_robot(timeout=0.1)
            if not success:
                print("Lost connection to robot")
                break

            # Calculate Y correction using sine wave
            t = time.time() - t_start
            y_correction = amplitude * np.sin(2 * np.pi * frequency * t)

            correction = np.array([0.0, y_correction, 0.0])
            egm.send_to_robot_path_corr(correction, age=1)

            # Match robot's sensor refresh rate of 48ms
            time.sleep(0.048)


if __name__ == "__main__":
    main()
