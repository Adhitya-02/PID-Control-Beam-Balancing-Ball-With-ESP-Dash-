# PID Control Beam Balancing Ball With ESP Dash

## Overview

This project involves implementing a PID (Proportional-Integral-Derivative) control system to balance a ball on a beam. The balancing parameters, namely Kp (Proportional Gain), Ki (Integral Gain), and Kd (Derivative Gain), can be dynamically adjusted through a web interface provided by ESP Dash.

## ESP Dash Webserver

ESP Dash serves as a webserver for configuring the PID control values. It simplifies the tuning process by allowing users to set and fine-tune the Kp, Ki, and Kd parameters in real-time. This web interface enhances the flexibility and ease of use of the PID control system, making it accessible for experimentation and optimization.

## Getting Started

To begin using this project, follow these steps:

1. **Install Dependencies:** Ensure that you have all the necessary dependencies installed on your system. Details can be found in the project documentation.

2. **Upload Code to ESP Device:** Upload the provided code to your ESP device. This code integrates the PID control algorithm and establishes a connection with the ESP Dash webserver.

3. **Connect to ESP Dash:** Once the code is running on your ESP device, connect to the ESP Dash web interface using a browser. The default address is typically http://esp-device-ip:port.

4. **Tune PID Parameters:** Use the ESP Dash interface to experiment with different Kp, Ki, and Kd values. Observe the impact on the ball balancing system in real-time.

## Project Structure

- **/src:** Contains the source code for the PID control system.
- **/web:** Includes files related to the ESP Dash web interface.

## Contributing

If you'd like to contribute to this project, please follow our [contribution guidelines](CONTRIBUTING.md). We welcome feedback, bug reports, and feature requests.

## License

This project is licensed under the [MIT License](LICENSE.md).

Feel free to reach out if you have any questions or need assistance with the project.
