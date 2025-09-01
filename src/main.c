/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
//#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-------------------- Defines ---------------------------------------*/
#define QUEUE_LENGTH 1
#define RED  	0
#define AMBER  	1
#define GREEN  	2
#define RED_LED_PIN  	GPIO_Pin_0
#define AMBER_LED_PIN  	GPIO_Pin_1
#define GREEN_LED_PIN  	GPIO_Pin_2
#define SPC_DATA_PIN GPIO_Pin_6
#define SPC_CLOCK_PIN GPIO_Pin_7
#define SPC_RESET_PIN GPIO_Pin_8
#define CAR_AVAILABLE 1
#define CAR_NOT_AVAILABLE 0

/*-------------------- Global values ---------------------------------------*/



/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
xQueueHandle xQueue_trafficFlowValue = 0;
xQueueHandle xQueue_newCarFlag = 0;

xTimerHandle xRedLedTimer;
xTimerHandle xAmberLedTimer;
xTimerHandle xGreenLedTimer;

/*--------------------------Helper functions---------------------------------*/


/**
 *	Init the traffic lights.
 */
static void TrafficLight_GPIO_Init() {

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //Enable the GPIO_LED Clock.

	// Configure the GPIO_LED pin
	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN | AMBER_LED_PIN | GREEN_LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // Set mode to output.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //Use pull-up for traffic lights.
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * Init GPIO for the Shift Register.
 */
static void SPC_GPIO_Init() {

	GPIO_InitTypeDef SPC_GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //Enable the GPIO_LED Clock.

	// Configure the pins for SPC.
	SPC_GPIO_InitStructure.GPIO_Pin = SPC_DATA_PIN | SPC_CLOCK_PIN | SPC_RESET_PIN;
	SPC_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	SPC_GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	SPC_GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //TODO check this
	SPC_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &SPC_GPIO_InitStructure);

	GPIO_SetBits(GPIOC, SPC_RESET_PIN); // Clear and reset the data.
}


static void SPC_Output(uint16_t value) {

	if (value == 0)  GPIO_ResetBits(GPIOC, SPC_DATA_PIN); // if no car present, set output low

	else GPIO_SetBits(GPIOC, SPC_DATA_PIN); // otherwise, car on the road, set output high

	GPIO_SetBits(GPIOC, SPC_CLOCK_PIN); // set clock high

	GPIO_ResetBits(GPIOC, SPC_CLOCK_PIN); // set clock low again
}

/**
 * Init GPIO for the potentiometer.
 */
static void Potentiometer_GPIO_Init() {

	GPIO_InitTypeDef ADC_GPIO_InitStructure; // Init a GPIO for ADC input.

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //Enable the GPIO Port C Clock.

	ADC_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	ADC_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; // Set analog mode.
//	ADC_GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	ADC_GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // no-pull for ADC.
//	ADC_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

	GPIO_Init(GPIOC, &ADC_GPIO_InitStructure);
}

/**
 * Init ADC for the potentiometer.
 */
static void Potentiometer_ADC_Init() {

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enable a clock for ADC.

	//Init ADC.
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // Default option.
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // Default option (value range 0 - 4095).
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // Disabled as we have only one channel.
	ADC_InitStructure.ADC_ExternalTrigConv = DISABLE; //Disabled.
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Disabled as we don't need external pins to trigger.
	ADC_InitStructure.ADC_NbrOfConversion = 1; // Perform a single conversion when start conversion is called.

	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE); // Configure ADC CMD.

	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles); // Configure ADC channel.
}

static uint16_t ADC_Read_Potentiometer() {

	uint16_t adcInputValue;

	ADC_SoftwareStartConv(ADC1); // Start ADC conversion.

	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // Wait for EoC end of conversion.

	adcInputValue = ADC_GetConversionValue(ADC1); // Get the ADC value.

	printf("ADC value = %d", adcInputValue);

	return adcInputValue;
}

void Amber_LED_Callback() {

	GPIO_ResetBits(GPIOC, AMBER_LED_PIN); // Turn off amber LED.

	GPIO_SetBits(GPIOC, RED_LED_PIN); // Turn on red LED.

	xTimerStart(xRedLedTimer, 0); // Start red LED Timer.
}

void Green_LED_Callback() {

	GPIO_ResetBits(GPIOC, GREEN_LED_PIN); // Turn off green LED.

	GPIO_SetBits(GPIOC, AMBER_LED_PIN); // Turn on amber LED.

	xTimerStart(xAmberLedTimer, 0); // Start amber LED Timer.
}

void Red_LED_Callback() {

	GPIO_ResetBits(GPIOC, RED_LED_PIN); // Turn off red LED.

	GPIO_SetBits(GPIOC, GREEN_LED_PIN); // Turn on green LED.

	xTimerStart(xGreenLedTimer, 0); // Start green LED Timer.
}

/*------------------------- FreeRTOS Tasks --------------------------------------------------------*/

static void Traffic_Display_Task(void *pvParameters) {

	uint16_t cars[19] = {0};

	uint16_t newCarFlag = 0;

	uint16_t trafficFullFlag = 0;

	while (1) {
//		printf("in SPU output task\n");

		xQueueReceive(xQueue_newCarFlag, &newCarFlag, pdMS_TO_TICKS(0));

		if (xTimerIsTimerActive(xGreenLedTimer)) { // In green light state

			for (int i = 18; i >= 0; i--) // Shift all cars.
				cars[i+1] = cars[i];

			cars[0] = newCarFlag; // Add a new car (or not).

		} else if (xTimerIsTimerActive(xRedLedTimer) || xTimerIsTimerActive(xAmberLedTimer)) { // In red/Amber light state

			for (int i = 18; i >= 8; i--) // Shift all cars after the stop line.
				cars[i+1] = cars[i];

			cars[8] = 0;

			trafficFullFlag = 0;

			// before traffic lights
			for (int i = 7; i >= 0; i--) {
				if (cars[i] == 0) {
					for (int j = i; j > 0; j--)
						cars[j] = cars[j-1];
					break;
				}
				if (i == 0)
					trafficFullFlag = 1;
			}

			if (trafficFullFlag != 1) 	//If array is not full, then add new car
				cars[0] = newCarFlag;
		}

		for (int16_t i = 18; i >= 0; i--) // Display all cars on LEDs.
			SPC_Output(cars[i]);

		vTaskDelay(1000);

	}
}

static void Traffic_Flow_Adjustment_Task(void* pvParameters) {

	uint16_t trafficFlowInt = 0;

	while (1) {

		trafficFlowInt = ADC_Read_Potentiometer() / 512; // Get the ADC value and divide by 512 - range (0 - 7).

		printf("ADC value in Digital: %d\n", trafficFlowInt);

		xQueueSendToBack(xQueue_trafficFlowValue, &trafficFlowInt, pdMS_TO_TICKS(0));

		vTaskDelay(10);
	}
}

static void Traffic_Generator_Task( void* pvParameters){

	uint16_t trafficFlowInt = 0;

	uint16_t newCarFlag = CAR_NOT_AVAILABLE; // Flag to determine whether or not to generate a car

	while (1) {

		xQueuePeek(xQueue_trafficFlowValue, &trafficFlowInt, pdMS_TO_TICKS(0));

//		if(newCarFlag < 1) newCarFlag++; // Adjust the car generation speed during low traffic flow.

		if (rand() % 8 <= trafficFlowInt) // If a randomly generated value falls into the trafficFlow interval, generate a car.
			newCarFlag = CAR_AVAILABLE;
		else // Otherwise, do not generate a car.
			newCarFlag = CAR_NOT_AVAILABLE;

//		printf("new car flag: %d\n", newCarFlag);
		xQueueSendToBack(xQueue_newCarFlag, &newCarFlag, pdMS_TO_TICKS(0));

		vTaskDelay(500);
	}
}

/**
 *	x	Green	red
	0	2000	4000
	1	2500	3800
	2	3000	3600
	3	3500	3400
	4	4000	3200
	5	4500	3000
	6	5000	2800
	7	5500	2600
 */
static void Traffic_Light_State_Control_Task(void *pvParameters){

	uint16_t newTrafficFlowInt = 4; // Set default value.

	uint16_t currentTrafficFlowInt = 0; // Set default value.

	xQueueReceive(xQueue_trafficFlowValue, &newTrafficFlowInt, pdMS_TO_TICKS(0)); // Get the the traffic flow value from Queue.

	uint16_t greenLightTime = 6000; // Set default value.

	uint16_t redLightTime = 3000; // Set default value.

	while(1){

		xQueueReceive(xQueue_trafficFlowValue, &newTrafficFlowInt, pdMS_TO_TICKS(0));

		if(newTrafficFlowInt != currentTrafficFlowInt){ // If the traffic flow value has changed, update the LED timers.

			greenLightTime = 500 * newTrafficFlowInt + 2000;

			redLightTime = 4000 - 200 * newTrafficFlowInt;

			if (xTimerIsTimerActive(xRedLedTimer)) {
				xTimerStop(xRedLedTimer, 0);
				xTimerChangePeriod(xRedLedTimer, redLightTime, 0);
				xTimerChangePeriod(xGreenLedTimer, greenLightTime, 0);
				xTimerStop(xGreenLedTimer, 0);

			} else if (xTimerIsTimerActive(xGreenLedTimer)) {

				xTimerChangePeriod(xRedLedTimer, redLightTime, 0);
				xTimerStop(xRedLedTimer, 0);
				xTimerStop(xGreenLedTimer, 0);
				xTimerChangePeriod(xGreenLedTimer, greenLightTime, 0);

			} else if (xTimerIsTimerActive(xAmberLedTimer)) {

				xTimerChangePeriod(xRedLedTimer, redLightTime, 0);
				xTimerStop(xRedLedTimer, 0);
				xTimerChangePeriod(xGreenLedTimer, greenLightTime, 0);
				xTimerStop(xGreenLedTimer, 0);
			}

			currentTrafficFlowInt = newTrafficFlowInt; // Update the current traffic flow value stored on this task.
		}

		vTaskDelay(100);
	}
}


/*
 * ------------------ Main ---------------------------------------------------------------
 */
int main(void)
{

//	printf("start in main\n");
	/* Initialization */
	TrafficLight_GPIO_Init();
	Potentiometer_GPIO_Init();
	Potentiometer_ADC_Init();
	SPC_GPIO_Init();

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQueue_trafficFlowValue = xQueueCreate( 	QUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint16_t ) );	/* The size of each item the queue holds. */

	xQueue_newCarFlag = xQueueCreate( 	QUEUE_LENGTH,		/* The number of items the queue can hold. */
								sizeof( uint16_t ) );	/* The size of each item the queue holds. */

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue_trafficFlowValue, "TrafficFlowQueue" );
	vQueueAddToRegistry( xQueue_newCarFlag, "TrafficCarsQueue" );

	xTaskCreate(Traffic_Flow_Adjustment_Task, "trafficFlowAdjustmentTask",configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Traffic_Generator_Task, "trafficGeneratorTask",	configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Traffic_Light_State_Control_Task,"trafficLightStateControlTask", configMINIMAL_STACK_SIZE, NULL, 1,	NULL);
	xTaskCreate(Traffic_Display_Task, "trafficDisplayTask",	configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	xRedLedTimer = xTimerCreate("redLedTimer", 3000, pdFALSE, (void *) 0, Red_LED_Callback);
	xGreenLedTimer = xTimerCreate("greenLedTimer", 6000, pdFALSE, (void *) 0, Green_LED_Callback);
	xAmberLedTimer = xTimerCreate("amberLedTimer", 2000, pdFALSE, (void *) 0, Amber_LED_Callback);

	xTimerStart(xGreenLedTimer, 0); // Start green LED.

	GPIO_SetBits(GPIOC, GREEN_LED_PIN); // Turn on green LED by default.

	vTaskStartScheduler(); /* Start the tasks and timer running. */

	return 0;
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

