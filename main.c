/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: 
*******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard includes. */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "uart_functions.h"

#define TIMER_PERIOD 300000000
uint32_t count1;
uint32_t count2;
uint8_t interval;
unsigned int UNLOCKED=1;
unsigned int LOCKED=0;
unsigned int state = 1;
unsigned int attempts = 0;

//Default Combination
int firstDigit = 2;
int secondDigit = 4;
int thirdDigit = 1;
int fourthDigit = 5;
int fifthDigit = 3;

int getCombination();

int
main (void)
{

    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer ();

	GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);


    /* Initialize the UART module. */
    initUART ();
    writeString ("Please Lock the Combinational Lock\n");

	//Sets DCO frequency at 12 MHz to get correct baud rate of 9600
	CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

	FlashCtl_setWaitState(FLASH_BANK0, 2);
	FlashCtl_setWaitState(FLASH_BANK1, 2);
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
	CS_setExternalClockSourceFrequency(32000, CS_48MHZ);
	CS_startHFXT(false);
	CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

	Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_16, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
	Timer32_setCount(TIMER32_0_BASE, TIMER_PERIOD);

	Timer32_startTimer(TIMER32_0_BASE, false);
//	if ((getCombination() == LOCKED) == 0) {
//		GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
//	}
//	else if ((getCombination() == UNLOCKED) == 0) {
//		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
//	}
    int userInput;
	readInt(userInput);




    while (1) {

		readInt(userInput);
		//writeInt(state);
		if (state == UNLOCKED && userInput == 1) {
		    state = LOCKED;
		    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
		    //writeInt(state);
		    writeString ("Please enter code to unlock the system:\n");

		    if ((getCombination() == LOCKED) == 0) {
		        attempts += 1;
				if (attempts >= 3) {
					UNLOCKED = 0;
					writeString("Enter Combination for the SUPERVISOR\n");
					writeString("You have only one attempt\n");
					if ((getCombination() == 1) == 0) {
						UNLOCKED = 1;
						attempts = 0;
					}
				}
		    }
		    else if ((getCombination() == UNLOCKED) == 0) {
		        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
				count1 = Timer32_getValue(TIMER32_0_BASE);

				while (interval <= 5) {
					count2 = Timer32_getValue(TIMER32_0_BASE);
					interval = (uint8_t)((count1 - count2) / 3000000);
					writeString("You Got 5 seconds to enter the combination again\n");
                    if ((getCombination() == UNLOCKED) == 0) {
						state = UNLOCKED;
                        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
					}
				}
		      }
			else {
				continue;
			}
		}
		else if (state == UNLOCKED && userInput == 2) {
		    GPIO_toggleOutputOnPin (GPIO_PORT_P1, GPIO_PIN0);
			writeString("Enter New Key\n");
			writeString("Enter First Digit:\n");
			readInt(firstDigit);
			writeString("Enter Second Digit:\n");
			readInt(secondDigit);
			writeString("Enter Third Digit:\n");
			readInt(thirdDigit);
			writeString("Enter Forth Digit:\n");
			readInt(fourthDigit);
			writeString("Enter Fifth Digit:\n");
			readInt(fifthDigit);

			unsigned int newfirstDigit;
			unsigned int newSecondDigit;
			unsigned int newThirdDigit;
			unsigned int newFourthDigit;
			unsigned int newFifthDigit;

			writeString("Enter Key Again to Confirm\n");

			writeString("Enter First Digit:\n");
			readInt(newfirstDigit);
			if (firstDigit == newfirstDigit) {
				firstDigit = newfirstDigit;
			}
			writeString("Enter Second Digit:\n");
			readInt(newSecondDigit);
			if (secondDigit == newSecondDigit) {
				secondDigit = newSecondDigit;
			}
			writeString("Enter Third Digit:\n");
			readInt(newThirdDigit);
			if (thirdDigit == newThirdDigit) {
				thirdDigit = newThirdDigit;
			}
			writeString("Enter Forth Digit:\n");
			readInt(newFourthDigit);
			if (fourthDigit == newFourthDigit) {
				fourthDigit = newFourthDigit;
			}
			writeString("Enter Fifth Digit:\n");
			readInt(newFifthDigit);
			if (fifthDigit == newFifthDigit) {
				fifthDigit = newFifthDigit;
			}

			GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
		}

	}
}



int getCombination()
{
	int currentState = -1; /* Our start state. */
	int input;
	while (1) {
		/* Get button pushed by user via UART. */
		readInt(input);
		
		if (input == firstDigit) {
			
				if (currentState == -1)
					currentState = firstDigit;
				else
					return 0;
				break;
		}
		else if (input == secondDigit) {
			
				if (currentState == firstDigit)
					currentState = secondDigit;
				else
					return 0;
					break;
		}
		else if (input == fifthDigit) {
			
				if (currentState == fourthDigit)
					return 1;
				else
					return 0; 
					break;
		}
		else if (input == thirdDigit) {
			
				if (currentState == secondDigit)
					currentState = thirdDigit;
				else
					return 0;
				break;
		}
		else if (input == fourthDigit) {
	
				if (currentState == thirdDigit)
					currentState = fourthDigit;
				else
					return 0;
				break;
		}
		else {
			return 0;
			break;
		}

	}
}



