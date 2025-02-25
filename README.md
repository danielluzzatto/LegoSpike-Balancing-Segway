# LegoSpike-Balancing-Segway
Balacing LegoSpike Robot using PID controller and LQR based on the pendulum on a cart model.
This project explores the control theory problem of stabilizing an inverted pendulum using the LEGO Spike Prime platform. Two control strategies were implemented and compared: PID Controller and LQR. The goal was to stabilize the pendulum and evaluate the performance of each method in theory and practice.

# Key Features
Platform: LEGO Spike Prime, including gyroscope, accelerometer, motor encoders, and motors.

# Control Methods:
- PID Controller: A simpler method, but faced challenges in this application. In particular, since the system is highly unstable, the tuning was particularly challenging 
- LQR Controller: Successfully stabilized the system, offering superior performance and robustness.

# State Estimation:
- A complementary filter was designed to fuse gyroscope and accelerometer data for accurate angle estimation. The sensor were tested with both static and dynamic tests.
- Motor encoders were used to calculate the position and linear velocity of the robot.

# Files Included
- Code Files: Python scripts for both PID and LQR implementations.
- Simulation Data: MATLAB/Simulink files for theoretical analysis and testing.
- Sensor Data: CSV files collected from tests on the robot.
- Report: Explanation of the methods, theory, simulations, results, and conclusions.

# How to Run the Code
Hardware Requirements:
- LEGO Spike Prime set, including motors, sensors, and hub.
- A compatible computer with the LEGO Spike app installed. The project can be run on other platforms such as Pybricks but the built-in lego functions might not be the same.
# Steps:
- Upload the Python files to the LEGO Spike hub.
- Ensure the hardware is assembled according to the instructions in the report.
- Run the desired script (PID or LQR) and observe the robot's behavior.

# Results Summary
The LQR controller proved to be significantly more effective than the PID controller in stabilizing the inverted pendulum. When testing the robot using LQR, the setup stabilized for up to 3 minutes. The system's response was smoother and more robust against disturbances, demonstrating the advantages of modern control theory in complex systems.

# License
This project is licensed under the MIT License.
