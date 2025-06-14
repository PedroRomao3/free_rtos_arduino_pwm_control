# FreeRTOS Arduino Project

This project integrates FreeRTOS with Arduino to manage multiple tasks for reading sensor data, processing control logic, and generating PWM output. The project utilizes the LSM9DS0 sensor for motion sensing and implements a Proportional (P) Controller for controlling outputs based on sensor readings.

## Project Structure

```
freertos-arduino-project
├── src
│   ├── main.cpp               # Entry point of the application
│   ├── tasks
│   │   ├── sensor_task.cpp     # Task for reading sensor data
│   │   ├── control_task.cpp     # Task for processing control logic
│   │   └── pwm_task.cpp         # Task for managing PWM output
│   ├── include
│   │   ├── sensor_task.h        # Header for sensor task
│   │   ├── control_task.h       # Header for control task
│   │   ├── pwm_task.h           # Header for PWM task
│   │   └── config.h             # Configuration constants
│   └── utils
│       ├── sensor_config.cpp     # Utility functions for sensor configuration
│       └── sensor_config.h       # Header for sensor configuration utilities
├── lib
│   └── FreeRTOS                 # FreeRTOS library files
├── platformio.ini               # PlatformIO configuration file
└── README.md                    # Project documentation
```

## Setup Instructions

1. **Clone the Repository**: Clone this repository to your local machine.
   
2. **Install PlatformIO**: Ensure you have PlatformIO installed in your development environment.

3. **Open the Project**: Open the `freertos-arduino-project` folder in PlatformIO.

4. **Install Dependencies**: The FreeRTOS library is included in the `lib` directory. Ensure all dependencies are resolved by PlatformIO.

5. **Build the Project**: Use the build command in PlatformIO to compile the project.

6. **Upload to Arduino**: Connect your Arduino board and upload the compiled firmware.

## Usage

- The project initializes the FreeRTOS scheduler and creates three tasks:
  - **Sensor Task**: Reads data from the LSM9DS0 sensor.
  - **Control Task**: Processes the sensor data and calculates the control output.
  - **PWM Task**: Manages the PWM output based on the control output.

- The tasks run concurrently, allowing for efficient data handling and control.

## Relevant Information

- Ensure that the correct I2C pins are configured for your specific Arduino board in the `main.cpp` file.
- Adjust the P Controller parameters in the control task as needed for your application.
- For further customization, refer to the header files in the `include` directory.

This project serves as a foundation for integrating FreeRTOS with Arduino for real-time applications involving sensor data processing and control systems.