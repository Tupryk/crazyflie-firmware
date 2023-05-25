/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * deck_servo.c - Deck driver for the servo deck
 */

#define DEBUG_MODULE "DeckServo"
#include "debug.h"

#include "deck.h"
#include "param.h"

#include "servo.h"

// linear servo
#define ratioOff 		0
#define ratioHalf 		128
#define ratioMax 		255

// linear actuator (right) (shaft looking up, arm at top == 100% length, arm at bottom == 0% length)
#define ratio_500us 	6 // 0.5 ms pulse at 50 Hz -> 100% length (GETS HOT, CRITICAL)
#define ratio_700us 	9 // 0.7 ms pulse at 50 Hz -> 95% length (OK, COOL)
#define ratio_750us 	10 // 0.75 ms pulse at 50 Hz -> 90% length
#define ratio_850us 	11 // 0.85 ms pulse at 50 Hz -> ...
#define ratio_950us 	12 // 0.95 ms pulse at 50 Hz -> ...
#define ratio_1000us 	13 // 1.0 ms pulse at 50 Hz -> ...
#define ratio_1500us 	19 // 1.5 ms pulse at 50 Hz -> ...
#define ratio_2000us 	26 // 2.0 ms pulse at 50 Hz -> 20% length
#define ratio_2300us 	29 // 2.3 ms pulse at 50 Hz -> 5% length
#define ratio_2500us 	32 // 2.5 ms pulse at 50 Hz -> 0% length (OK, COOL)

// ratio limits of the linear actuators (top == shaft end)
#define ratioRightTop 		ratio_700us
#define ratioRightBottom 	ratio_2300us
#define ratioLeftTop 		ratio_2300us
#define ratioLeftBottom 	ratio_850us

// normal servo info
//  500 us -> 190 deg
// 1000 us -> 135 deg
// 1500 us -> 100 deg
// 2000 us ->  60 deg
// 2500 us ->  10 deg

uint8_t ratio = 0;

uint16_t frequencyDefault = 50;
uint16_t frequency = 50;

uint8_t state = 0;

static void testServo(void);
static void activateServo(void);
static void deactivateServo(void);
static void setRatioCallback(void);
static void setFrequencyCallback(void);

static void initializeDeckServo()
{
	DEBUG_PRINT("Initializing servo deck!\n");

	// initialize the PWM pin TX1
    servoInit();

    DEBUG_PRINT("Servo deck initialized successfully!\n");

	// test the servo deck
	testServo();
}

static void testServo(void)
{
	DEBUG_PRINT("Starting servo deck test!\n");

	servoSetRatio(ratioRightBottom);
	servoSetFreq(frequencyDefault);

	DEBUG_PRINT("Ending servo deck test!\n");
}

static void activateCallback(void)
{
	if (state) {
		activateServo();
	} else {
		deactivateServo();
	}
}

static void activateServo()
{	
	DEBUG_PRINT("Activating servo!\n");

	servoSetRatio(ratio_1000us);
	servoSetFreq(frequencyDefault);
}

static void deactivateServo()
{
	DEBUG_PRINT("Deactivating servo!\n");

	servoSetRatio(ratio_1000us);
	servoSetFreq(frequencyDefault);
}

static void setRatioCallback(void)
{	
	DEBUG_PRINT("Setting servo ratio to %d!\n", ratio);

	servoSetRatio(ratio);
}

static void setFrequencyCallback(void)
{
	DEBUG_PRINT("Setting servo frequency to %d!\n", frequency);

	servoSetFreq(frequency);
}

static const DeckDriver driverServo = {
	.vid = 0,
  	.pid = 0,
	.name = "deckServo",

	.usedPeriph = DECK_USING_PA2 | DECK_USING_TIMER5,
	.usedGpio = 0,
	
	.init = initializeDeckServo,
};

DECK_DRIVER(driverServo);

// servo deck parameters
PARAM_GROUP_START(servo)

PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, activate, &state, &activateCallback)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, setRatio, &ratio, &setRatioCallback)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT16, setFrequency, &frequency, &setFrequencyCallback)

PARAM_GROUP_STOP(servo)