from egm import EGM


def main() -> None:
    egm = EGM()

    while True:
        success, state = egm.receive_from_robot()
        if success:
            print(state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z)


if __name__ == "__main__":
    main()
