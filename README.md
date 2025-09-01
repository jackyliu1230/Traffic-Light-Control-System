This project is a FreeRTOS-based traffic light simulation running on an STM32F4 microcontroller.

The system uses tasks, queues, and software timers provided by FreeRTOS to coordinate multiple components:

a) Traffic Flow Adjustment Task: Reads a potentiometer value via ADC to estimate traffic flow intensity.

b)  Traffic Generator Task: Simulates incoming cars based on the traffic flow setting.

c)  Traffic Light State Control Task: Dynamically adjusts red/green light durations depending on traffic conditions.

d)  Traffic Display Task: Displays the movement of cars through a shift register and LEDs according to traffic light states.

The LEDs (Red, Amber, Green) represent the traffic signals, while the simulated "cars" are shifted through registers to visualize flow. This project demonstrates how real-time scheduling, synchronization, and hardware interfacing can be achieved using FreeRTOS.
