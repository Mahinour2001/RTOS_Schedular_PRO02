/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

#include "queue.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

char runTimeStatsBuffer[250];

TaskHandle_t Button_1_TaskHandler = NULL;
TaskHandle_t Button_2_TaskHandler = NULL;
TaskHandle_t Periodic_TransmitterHandler = NULL;
TaskHandle_t Uart_ReceiverHandler = NULL;
TaskHandle_t Load1_Handler = NULL;
TaskHandle_t Load2_Handler = NULL;
TaskHandle_t Analysis_Handler  = NULL;

QueueHandle_t xQueue_button ;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

void Button_1_Task( void * pvParameters )
{
	
	pinState_t  Button1_CurrentState = PIN_IS_HIGH;
	pinState_t  Button1_NewState ;
	char* Button1String;
	TickType_t    xLastWakeTime = xTaskGetTickCount();
	
	
    for( ;; )
    {
			Button1_NewState=GPIO_read(PORT_0,PIN0);
			if(Button1_NewState != Button1_CurrentState)
			{
				//rissing edge
				if(Button1_NewState == PIN_IS_HIGH)
				{
					if ( xQueue_button != 0 )
				{
					Button1String="Button1_RissingEdge\n";
					xQueueSend( xQueue_button, ( void * ) &Button1String, ( TickType_t ) 0 );
				}
			}
				//falling edge
				else if(Button1_NewState == PIN_IS_LOW)
				{
					if ( xQueue_button != 0 )
				{
					Button1String="Button1_FallingEdge\n";
					xQueueSend( xQueue_button, ( void * ) &Button1String, ( TickType_t ) 0 );
				}
					
					
				}
				Button1_CurrentState = Button1_NewState;
				
			}
			
			GPIO_write(PORT_0,PIN4,PIN_IS_LOW);
			vTaskDelayUntil(&xLastWakeTime,50 );
			GPIO_write(PORT_0,PIN4,PIN_IS_HIGH);
			
			
			
			//idle task
	    GPIO_write(PORT_0,PIN3,PIN_IS_LOW);

			
    }
}
	
void Button_2_Task( void * pvParameters )
{
	pinState_t  Button2_CurrentState = PIN_IS_HIGH;
	pinState_t  Button2_NewState ;
	char* Button2String;
	TickType_t    xLastWakeTime = xTaskGetTickCount();
	
    for( ;; )
    {
			Button2_NewState=GPIO_read(PORT_0,PIN2);
			if(Button2_NewState != Button2_CurrentState)
			{
				//rissing edge
				if(Button2_NewState == PIN_IS_HIGH)
				{
					if ( xQueue_button != 0 )
				{
					Button2String="Button2_RissingEdge\n";
					xQueueSend( xQueue_button, ( void * ) &Button2String, ( TickType_t ) 0 );
				}
			}
				//falling edge
				else if(Button2_NewState == PIN_IS_LOW)
				{
					if ( xQueue_button != 0 )
				{
					Button2String="Button2_FallingEdge\n";
					xQueueSend( xQueue_button, ( void * ) &Button2String, ( TickType_t ) 0 );
				}
					
					
				}
				Button2_CurrentState = Button2_NewState;
				
			}
			
			GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
			vTaskDelayUntil(&xLastWakeTime, 50 );
			GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
			
			
			
			//idle task
	    GPIO_write(PORT_0,PIN3,PIN_IS_LOW);

			
    }
	  
}
void Periodic_Transmitter( void * pvParameters )
{
	
	char* Periodic_Transmitter_String ;
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	
    for( ;; )
    {
			
					if ( xQueue_button != 0 )
				{
					Periodic_Transmitter_String="Periodic_Transmitter_String\n";
					xQueueSend( xQueue_button, ( void * ) &Periodic_Transmitter_String, ( TickType_t ) 0 );
				}
			
					GPIO_write(PORT_0,PIN6,PIN_IS_LOW);
		    	vTaskDelayUntil(&xLastWakeTime, 100 );
			    GPIO_write(PORT_0,PIN6,PIN_IS_HIGH);
				
				//idle task
	    GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
				
				
			}
			
		
			
			
			
			

			
  }


void Uart_Receiver( void * pvParameters)
{
	char* Uart_Receiver_String ;
	TickType_t    xLastWakeTime = xTaskGetTickCount();
	
    for( ;; )
    {
			
					if ( xQueueReceive( xQueue_button,
                         &( Uart_Receiver_String ),
                         ( TickType_t ) 10 ) == pdPASS )
				{
					vSerialPutString((const signed char*)Uart_Receiver_String,20);
					
					
				}
				
			
				GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
		   	vTaskDelayUntil(&xLastWakeTime, (const TickType_t)20 );
		  	GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
			
			
			
			//idle task
	    GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
				
			}
			
			
	
}
	  
void Load_1_Simulation( void * pvParameters )
{
	int i=0;
	TickType_t    xLastWakeTime = xTaskGetTickCount();
	//GPIO_write(PORT_0,PIN8,PIN_IS_HIGH);
	 for( ;; )
  {	
		for( i=0;i<35000;i++ )
	{
	   	i=i;
		
		
	}
		    GPIO_write(PORT_0,PIN8,PIN_IS_LOW);
		   	vTaskDelayUntil(&xLastWakeTime, 10 );
		  	GPIO_write(PORT_0,PIN8,PIN_IS_HIGH);
			
			
			
			//idle task
	    GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	//GPIO_write(PORT_0,PIN8,PIN_IS_LOW);
  }
}

void Load_2_Simulation( void * pvParameters )
{
	int i=0;
	TickType_t    xLastWakeTime = xTaskGetTickCount();
	//GPIO_write(PORT_0,PIN8,PIN_IS_HIGH);
	 for( ;; )
  {	for( i=0;i<90000;i++ )
	{
		   i=i;
		
		
	}
		    GPIO_write(PORT_0,PIN9,PIN_IS_LOW);
		   	vTaskDelayUntil(&xLastWakeTime, 100 );
		  	GPIO_write(PORT_0,PIN9,PIN_IS_HIGH);
			
			
			
			//idle task
	    GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	//GPIO_write(PORT_0,PIN8,PIN_IS_LOW);
  }
	
}
void Analysis (void * parameters)
{
TickType_t xLastWakeTime;
 const TickType_t xFrequency = 120;
 // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();


    for (;;)
    {       
            vTaskGetRunTimeStats(runTimeStatsBuffer);
            xSerialPutChar('\n');
            vSerialPutString(runTimeStatsBuffer,100);
            vTaskDelayUntil(&xLastWakeTime,xFrequency);

    }
}
void vApplicationTickHook( void )
{
	GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
	
}

void vApplicationIdleHook( void )
{
	GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
	//GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
	
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	/* Create the task, storing the handle. */
xQueue_button = xQueueCreate( 10, sizeof( unsigned long ) );

    if( xQueue_button == NULL )
    {
        /* Queue was not created and must not be used. */
    }
	
		
xTaskPeriodicCreate(
                    Button_1_Task,       /* Function that implements the task. */
                    "Button_1_Task",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_1_TaskHandler ,
                       50										); 
										
xTaskPeriodicCreate(
                    Button_2_Task,       /* Function that implements the task. */
                    "Button_2_Task",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_2_TaskHandler ,
										50); 	
										
xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Periodic_Transmitter",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Periodic_TransmitterHandler ,
                       100										); 										
xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Uart_Receiver",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Uart_ReceiverHandler ,
                       20										);   
xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load_1_Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load1_Handler ,
                       10										);   
xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load_2_Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load2_Handler ,
                       100										);   										
										
xTaskPeriodicCreate(
                    Analysis,       
                    "Analysis",         
                    100,      
                    ( void * ) 1,    
                    1,   
                    &Analysis_Handler,
                                        120);									
										
	
    /* Create Tasks here */


	
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}


static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


