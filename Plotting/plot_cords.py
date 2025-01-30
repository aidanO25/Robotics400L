import matplotlib.pyplot as plt
import numpy as np
import serial
import time

def main():
    plt.ion()  # interactive
    fig, ax = plt.subplots()
    obstacles_scatter = ax.scatter([], [], color='blue')  # obstacles
    robot_scatter = ax.scatter([], [], color='red')  # robot
    ax.set_xlim(-100, 100)  # Set initial x-axis limits
    ax.set_ylim(-100, 100)  # Set initial y-axis limits
    
    port = '/dev/cu.usbmodem14201'
    try:
        ser = serial.Serial(port, 57600, timeout=1)  # Open serial port
        print("Serial port opened.")
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        return

    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if '|' in line:  
                    robot_data, obstacle_data = line.split('|')
                    
                    robot_coords = robot_data.split(',')
                    obstacle_coords = obstacle_data.split(',')
                    
                    if len(robot_coords) == 2 and len(obstacle_coords) == 2:
                        robot_x, robot_y = map(int, robot_coords)
                        obstacle_x, obstacle_y = map(int, obstacle_coords)

                        # Update robot scatter plot
                        robot_scatter.set_offsets([robot_x, robot_y]) # update robot pos
                        
                        # Update obstacles scatter plot
                        old_offsets = np.array(obstacles_scatter.get_offsets())
                        new_offsets = np.vstack([old_offsets, [obstacle_x, obstacle_y]]) # combine 
                        obstacles_scatter.set_offsets(new_offsets) # set
                    
                    plt.draw()
                    plt.pause(0.01)  # Short pause to update the plot
        except ValueError as e:
            print(f"Error processing line '{line}': {e}")
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            ser.close()
            break

    ser.close()
    plt.ioff()  # Turn off interactive mode
    plt.show()

if __name__ == "__main__":
    main()