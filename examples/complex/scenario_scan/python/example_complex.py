from ABBRobotEGM import EGM
import matplotlib.pyplot as plt
import numpy as np
import time


def main() -> None:
    # Initialize plot with proper layout for colorbar
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()
    fig.set_size_inches(10, 8)  # Adjust figure size

    # Define scan area dimensions
    width, height = 200, 250  # mm
    resolution = 5  # 5mm² per pixel
    grid_width = width // resolution
    grid_height = height // resolution

    # Create grid to store sensor values
    grid = np.full((grid_height, grid_width), np.nan)  # Initialize with NaN

    # Create image display with interpolation='nearest' for faster rendering
    img = ax.imshow(grid, extent=[0, width, 0, height],
                    origin='lower', cmap='viridis',
                    vmin=0.3, vmax=0.6, interpolation='nearest')

    # Create colorbar with specific axes
    cbar = fig.colorbar(img, ax=ax, label='Sensor Value')

    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')
    ax.set_title('Scan Area with Sensor Values')
    ax.grid(True)
    ax.set_aspect('equal')

    # Set a fixed update rate
    update_interval = 0.1  # seconds
    last_update = time.time()

    with EGM() as egm:
        while True:
            success, state = egm.receive_from_robot()
            if not success:
                print("Failed to receive from robot")
                break

            # Tell the robot, python script is ready to receive data
            egm.send_to_robot(digital_signal_to_robot=(True))

            # Get current position and sensor value
            actual_x = int(state.cartesian.pos.x // resolution)
            actual_y = int(state.cartesian.pos.y // resolution)
            sensor_value = state.rapid_from_robot[0]

            # Update grid if position is within bounds
            if 0 <= actual_x < grid_width and 0 <= actual_y < grid_height:
                # Store the sensor value in the grid
                grid[actual_y, actual_x] = sensor_value

                # Update plot at fixed intervals
                current_time = time.time()
                if current_time - last_update >= update_interval:
                    img.set_array(grid)
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    last_update = current_time

            # print(f"Position: ({actual_x}, {actual_y}) mm, Value: {sensor_value}")


if __name__ == "__main__":
    main()
