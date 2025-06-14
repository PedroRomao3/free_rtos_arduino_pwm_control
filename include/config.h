#ifndef CONFIG_H
#define CONFIG_H

// FreeRTOS task priorities
#define SENSOR_TASK_PRIORITY 1
#define CONTROL_TASK_PRIORITY 2
#define PWM_TASK_PRIORITY 3

// Stack sizes for each task (in words)
#define SENSOR_TASK_STACK_SIZE 256
#define CONTROL_TASK_STACK_SIZE 256
#define PWM_TASK_STACK_SIZE 256

// Other configuration constants
#define LOOP_DELAY 10 // Delay in milliseconds for the main loop

#endif // CONFIG_H