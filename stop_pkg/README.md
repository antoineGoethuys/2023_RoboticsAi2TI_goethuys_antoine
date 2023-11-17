# README

This README file provides instructions on how to run the provided ROS2 Python code, explanations of the implemented functionalities, and any dependencies required.

## Dependencies

Before running the code, make sure you have the following dependencies installed:

- ROS2 (Robot Operating System 2): Follow the installation instructions on the ROS2 website (https://docs.ros.org/en/foxy/Installation.html) to install ROS2 on your system.

- Python 3: Ensure that you have Python 3 installed on your system. ROS2 uses Python 3 for its development.

## Running the Code

Follow these steps to run the code:

1. Open a terminal and source your ROS2 installation:

    ```bash
    source /path/to/ros2/installation/setup.bash
    ```

2. Navigate to the directory containing the Python script.

3. Run the Python script:

    ```bash
    python script_name.py
    ```

    Replace `script_name.py` with the name of your Python script.

4. The script will start running, and you should see output indicating the laser scan information and the robot's motion logic.

## Code Explanation

### Node Class: `Stop`

- The `Stop` class is a ROS2 node that combines both a publisher and a subscriber. It listens to laser scan data and publishes velocity commands to control a robot.

- The node has the following components:

    - **Publisher**: Publishes velocity commands to the 'cmd_vel' topic.

    - **Subscriber**: Subscribes to laser scan data from the '/scan' topic.

    - **Timer**: Defines a timer with a period of 0.5 seconds, calling the `motion` function.

    - **Variables**:
        - `laser_forward`: Stores the distance from the front of the robot obtained from laser scan data.
        - `cmd`: An instance of the `Twist` message type for controlling linear and angular velocities.

### Callback Function: `laser_callback`

- This function is called when new laser scan data is received.

- It extracts the distance at 0° from the laser scan data and stores it in the `laser_forward` variable.

### Motion Function: `motion`

- This function is called at regular intervals by the timer.

- It prints the received laser scan data and applies a simple motion logic based on the distance obtained.

    - If the distance is greater than 2 units, the robot moves forward with linear velocity of 0.2.
    - If the distance is between 0.2 and 2 units, the robot moves forward with linear velocity of 0.1.
    - If the distance is less than 0.2 units, the robot stops.

- The calculated velocity command is published to the 'cmd_vel' topic.

### Main Function: `main`

- Initializes ROS communication using `rclpy.init`.

- Creates an instance of the `Stop` class, which sets up the publisher, subscriber, and timer.

- Spins the node to keep it active until a shutdown signal is received (e.g., `Ctrl+C`).

- Explicitly destroys the node and shuts down ROS communication when the node is stopped.

### Running the Script

- The `if __name__ == '__main__':` block ensures that the `main` function is called when the script is executed directly.

## Additional Notes

- Make sure your ROS2 environment is properly set up before running the script.

- Ensure that the robot's sensor data is being published to the '/scan' topic, and the velocity commands are being listened to on the 'cmd_vel' topic.

- Adjust the motion logic in the `motion` function as needed for your specific robot and sensor configuration.

- This README assumes basic familiarity with ROS2 concepts and assumes that the code is tailored to your specific robot and sensor setup.
