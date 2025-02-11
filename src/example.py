from egm import EGM


def main() -> None:
    egm = EGM()

    while True:
        success, state = egm.receive_from_robot()
        if success:
            egm.debug_print_robot_message(state.robot_message)


if __name__ == "__main__":
    main()
