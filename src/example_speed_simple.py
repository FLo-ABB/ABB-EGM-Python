from egm import EGM
import numpy as np
import time

# Robot Module #


def calculate_span_speed(positions, timestamps):
    """
    Calculate speed using span method (oldest to newest sample).

    :param positions: List of position values
    :param timestamps: List of corresponding timestamps
    :return: Speed in units/second, or 0.0 if insufficient data
    """
    if len(positions) < 2 or len(timestamps) < 2:
        return 0.0

    time_span = timestamps[-1] - timestamps[0]
    position_span = positions[-1] - positions[0]

    return position_span / time_span if time_span > 0 else 0.0


def main() -> None:
    """Simple speed guidance example with span-based speed calculation."""
    egm = EGM()

    start_time = time.time()
    duration = 5.0

    # Variables for span-based speed calculation
    position_history = []
    time_history = []
    max_samples = 8  # Number of samples for averaging
    last_print_time = 0.0

    while time.time() - start_time < duration:
        success, state = egm.receive_from_robot()
        if success:
            # Get current position and orientation
            current_pos = np.array([
                state.cartesian.pos.x,
                state.cartesian.pos.y,
                state.cartesian.pos.z
            ])
            current_orient = np.array([
                state.cartesian.orient.u0,  # w
                state.cartesian.orient.u1,  # x
                state.cartesian.orient.u2,  # y
                state.cartesian.orient.u3   # z
            ])

            # Create a linear speed reference
            speed_ref = egm.create_linear_speed_ref(vz=50)  # 10 mm/s upward

            # Send current position with speed reference
            egm.send_to_robot_cart(current_pos, current_orient, speed_ref)

            # Calculate and print progress
            current_time = time.time()
            elapsed = current_time - start_time

            # Store current position and time for span-based calculation
            position_history.append(current_pos[2])  # Store Z position
            time_history.append(current_time)

            # Keep only the last max_samples entries
            if len(position_history) > max_samples:
                position_history.pop(0)
                time_history.pop(0)

            # Calculate span-based speed using helper function
            span_speed = calculate_span_speed(position_history, time_history)

            # Print every 0.2 seconds
            if elapsed - last_print_time >= 0.2:
                print(f"Time: {elapsed:.1f}s, Z pos: {current_pos[2]:.1f}mm, "
                      f"Speed: {span_speed:.1f}mm/s")
                last_print_time = elapsed

        else:
            print("Failed to receive from robot")
            break

    print("Speed guidance example completed!")


if __name__ == "__main__":
    main()
