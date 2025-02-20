from egm import EGM
from abb_data import Pos, Orientation, Euler, Pose


def main() -> None:
    egm = EGM()

    while True:
        success, state = egm.receive_from_robot()
        if success:
            # Move joint 1 by 1 degree
            joint_angles = state.joint_angles.copy()
            joint_angles[0] -= 1

            # Create a new EGMRobotState instance with the updated joint_angles
            new_state = state._replace(joint_angles=joint_angles)
            with open("message_send_to_robot.txt", "w") as f:
                f.write(str(new_state))

            # Extract joint_angles from the new_state and send to robot
            egm.send_to_robot(new_state.joint_angles)


if __name__ == "__main__":
    main()
