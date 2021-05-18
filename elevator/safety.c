/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
 */

/**
 * This file defines the safety module, which observes the running
 * elevator system and is able to stop the elevator in critical
 * situations
 */

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "semphr.h"
#include <stdio.h>

#include "global.h"
#include "assert.h"

#define POLL_TIME (10 / portTICK_RATE_MS)
#define MAX_IDLE  (1000 / portTICK_RATE_MS)
#define MIN_IDLE	(30 / portTICK_RATE_MS)
#define MOTOR_WAIT_TIME (100 / portTICK_RATE_MS)

#define MOTOR_UPWARD   (TIM3->CCR1)
#define MOTOR_DOWNWARD (TIM3->CCR2)
#define MOTOR_STOPPED  (!MOTOR_UPWARD && !MOTOR_DOWNWARD)
#define ABS(a)	  	 	 ((a) < 0 ? -(a) : (a))
#define DOOR_MASK			0
#define ARRIVED_MASK	1
#define STOP_MASK			2

#define STOP_PRESSED  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)
#define AT_FLOOR      GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)
#define DOORS_CLOSED  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)

typedef struct
{
	u8 doors_have_opened:1;
	u8 arrived_at_floor:1;
	u8 stop_recently_pressed:1;
	u8 idle:1;
} bits;

static portTickType xLastWakeTime;

static void check(u8 assertion, char *name) {
  if (!assertion) {
    printf("SAFETY REQUIREMENT %s VIOLATED: STOPPING ELEVATOR\n", name);
    for (;;) {
	  setCarMotorStopped(1);
  	  vTaskDelayUntil(&xLastWakeTime, POLL_TIME);
    }
  }
}

static void safetyTask(void *params) {
  s16 timeSinceStopPressed = -1;
	s16 timeSinceLastSpeedUpdate = 0;			// time between 10 cm
	s16 timeSinceLastPositionChange = 0;	// how long since last change in position
	s16 timeBetweenCentimeters = 0;				// time between 1 cm, used for measuring low speeds
	bits boolbits = {0,0,0,0};		// reduce space by having all bools as single bytes
	vs32 pos = 0;
	vs32 last_pos = 0;		// used for measuring time between 10 cm
	vs32 prev_pos = 0;		// used for measuring time between 1 cm
	vs32 distance;				// up to 10 cm
	s16 idle_time;

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Environment assumption 1: the doors can only be opened if
	//                           the elevator is at a floor and
    //                           the motor is not active
	
	check((AT_FLOOR && MOTOR_STOPPED) || DOORS_CLOSED,
	      "env1");

	// get position, used for some of the checks
	pos = getCarPosition();
		
		// Environment assumption 2: elevator must not move faster than 50 cm/s (a duty of 10000)
			
	distance = ABS(pos - last_pos);
	if (distance >= 10)
	{
		// 200 ms at 50 cm/s is 10 cm, but there's a margin of error as this task updates every 10 ms
		// if a position update was missed, then elevator goes too fast (10 ms is 100 cm/s)
		check(timeSinceLastSpeedUpdate >= 180 && distance == 10, "env2");
		last_pos = pos;
		timeSinceLastSpeedUpdate = 0;
	}
	else if (!MOTOR_STOPPED)
		timeSinceLastSpeedUpdate += POLL_TIME;		// update counter if moving
	else
	{
		timeSinceLastSpeedUpdate = 0;			// if not moving, stop checking
		last_pos = pos;
	}
	
	// Environment assumption 2 (continued): elevator must move slowly enough that we can measure distance
	// measure time between each cm (used for measuring low speeds)
	// also used by env2
	if (pos != prev_pos)
	{
		check(ABS(pos - prev_pos) == 1, "env2");		// additional speed check, must catch every position change or we go too fast
		timeBetweenCentimeters = timeSinceLastPositionChange;		// update how much time it takes between centimeters
		timeSinceLastPositionChange = 0;
		prev_pos = pos;
	}
	else if (!MOTOR_STOPPED)
		timeSinceLastPositionChange += POLL_TIME;
	else
		timeSinceLastPositionChange = 0;
		
		// fill in environment assumption 3: floors must be at 0, 400 and 800 resp. (+/- 1cm)
	check((!AT_FLOOR || ((pos <= FLOOR_1_UB) || 
											 (pos >= FLOOR_2_LB && pos <= FLOOR_2_UB) || 
											 (pos >= FLOOR_3_LB))), "env3");

		// fill in your own environment assumption 4: if elevator has arrived at floor, doors must open at some point
		// in other words, if we leave the floor, then the doors must have been open at some point
	if (AT_FLOOR && MOTOR_STOPPED)
	{
		// we have arrived at a floor
		boolbits.arrived_at_floor = 1;
		// if doors have opened, set flag
		if (!DOORS_CLOSED)
			boolbits.doors_have_opened = 1;
	}
	// check occurs once the elevator starts moving after having been at a floor
	if (boolbits.arrived_at_floor && !MOTOR_STOPPED)
	{
		check(boolbits.doors_have_opened, "env4");
		boolbits.arrived_at_floor = 0;
		boolbits.doors_have_opened = 0;
	}
	
    // System requirement 1: if the stop button is pressed, the motor is
	//                       stopped within 1s

	// added some logic for safety req 4
	// if timeSinceStopPressed is positive, it measure time since stop was pressed,
	// if negative it measures time since stop was released
	// the measuring of release is stopped after 1 second to prevent overflow
	if (STOP_PRESSED) {
		boolbits.stop_recently_pressed = 1;
	  if (timeSinceStopPressed < 0)
	    timeSinceStopPressed = 0;
      else
	    timeSinceStopPressed += POLL_TIME;

      check(timeSinceStopPressed * portTICK_RATE_MS <= 1000 || MOTOR_STOPPED,
	        "req1"); 
	} else if (boolbits.stop_recently_pressed) {
		if (timeSinceStopPressed > 0)
			timeSinceStopPressed = 0;
		else if (-timeSinceStopPressed * portTICK_RATE_MS >= 1000)
		{
			boolbits.stop_recently_pressed = 0;
			timeSinceStopPressed = -1;
		}
		else
			timeSinceStopPressed -= POLL_TIME;
	}

    // System requirement 2: the motor signals for upwards and downwards
	//                       movement are not active at the same time

    check(!MOTOR_UPWARD || !MOTOR_DOWNWARD,
          "req2");

	// fill in safety requirement 3
	check((pos >= FLOOR_1 && pos <= FLOOR_3), "req3");

	// Safety requirement 4:
	// if 1 s has passed since elevator moved, it can be considered to have halted
	// it can take a bit of time for the elevator to move after stop button pressed
	//   take account of that by using stop_recently_pressed instead of STOP_PRESSED
	check(boolbits.stop_recently_pressed || AT_FLOOR || timeBetweenCentimeters * portTICK_RATE_MS < 1000, "req4");
	
	// fill in safety requirement 5: when elevator arrives at a floor, it must wait for at least 1 s
	// if elevator arrives at a floor, set it to idle and start the timer
	if (AT_FLOOR && MOTOR_STOPPED)
	{
		// not yet idle, enter idle mode
		if (!boolbits.idle)
		{
			boolbits.idle = 1;
			idle_time = 0;
		}
		// we are idle, increment counter
		else
			idle_time += POLL_TIME;
	}
	check(MOTOR_STOPPED || !boolbits.idle || idle_time >= MAX_IDLE, "req5");
	// we can start moving after the check (otherwise idle will be 0 before the condition has been checked)
	if (!(AT_FLOOR && MOTOR_STOPPED))
		boolbits.idle = 0;

	// fill in safety requirement 6: if approaching ground floor or top floor, elevator must move slowly	
	check(timeBetweenCentimeters >= 300 ||
					((!MOTOR_DOWNWARD || pos > FLOOR_1_UB + 1) &&
					 (!MOTOR_UPWARD || pos < FLOOR_3_LB - 1)),
				"req6");

	// fill in safety requirement 7
	check(1, "req7");

	vTaskDelayUntil(&xLastWakeTime, POLL_TIME);
  }

}

void setupSafety(unsigned portBASE_TYPE uxPriority) {
  xTaskCreate(safetyTask, "safety", 100, NULL, uxPriority, NULL);
}
