/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
 */

/**
 * The planner module, which is responsible for consuming
 * pin/key events, and for deciding where the elevator
 * should go next
 */

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "global.h"
#include "planner.h"
#include "assert.h"

// precondition: pos is a valid floor position
#define GET_FLOOR(pos)		((((pos) + 1) / FLOOR_2) + 1)

#define TOP_FLOOR			3
#define BOTTOM_FLOOR	1

#define IDLE_TIME			(2000 / portTICK_RATE_MS)
#define POLL_TIME			(50 / portTICK_RATE_MS)

typedef enum {
  Unknown = 0, Up = 1, Down = 2
} Direction;

// updateTargetFloor()
// determines which floor to go to based on the floor indicators, 
// current target floor and direction of elevator
// parameters
//		floor -					 array containing status of each floor indicator (1 if in queue, 0 otherwise)
//		current_target - the current target floor of the elevator (only relevant if 2)
//		dir -						 direction of elevator
// returns
//		target of floor
u8 updateTargetFloor(u8* floor, u8 current_target, Direction dir)
{
	vs32 pos = getCarPosition();
	// go to floor 2 if elevator is not too close
	if (floor[1] && ((dir == Down && pos > FLOOR_2_UL) || (dir == Up && pos < FLOOR_2_LL)))
	{
		printf("Going to floor 2\n");
		setCarTargetPosition(FLOOR_2);
		return 2;
	}
	// go to floor 3 if floor 2 is not the target and elevator is going up or if floor 3 is only active floor
	if (floor[2] && ((dir == Up && current_target != 2) || (!floor[0] && !floor[1])))
	{
		printf("Going to floor 3\n");
		setCarTargetPosition(FLOOR_3);
		return 3;
	}
	// go to floor 1 if floor 2 is not the target and elevator is going down or floor 1 is only active floor
	if (floor[0] && ((dir == Down && current_target != 2) || (!floor[1] && !floor[2])))
	{
		printf("Going to floor 1\n");
		setCarTargetPosition(FLOOR_1);
		return 1;
	}
	// if no floor button is activated, stay put
	if (!floor[0] && !floor[1] && !floor[2])
		return 0;
	// if none of the conditions above apply, just keep going
	return current_target;
}

static void plannerTask(void *params) {

  // ...
	PinEvent event;			// event from event queue
	vs32 pos;						// used to store position of elevator
	u8	arrived = 1;		// 1 if elevator is supposed to wait at floor, otherwise 0
	s16 idle_time = 0;	// time elevator has waited at floor
	u8 floor_indicator[3] = {0,0,0};	// indicator for each floor (1 if floor is in queue, 0 otherwise)
	u8 target = 0;			// target floor of elevator
	u8 current_floor = 1;	// current floor of elevator (NOTE: undefined behaviour if elevator is not at floor)
	u8 doors_closed = 0;
	Direction direction = Up;		// direction of elevator
	
	for (;;)
	{
		if (uxQueueMessagesWaiting(pinEventQueue) > 0)	// is there a message waiting?
		{
			xQueueReceive(pinEventQueue, &event, portMAX_DELAY);
			switch(event)
			{
				// TO_FLOOR_x - set indicator of floor
				// if not waiting at a floor, update target of elevator
				case TO_FLOOR_1:
					floor_indicator[0] = 1;
					if (!arrived) target = updateTargetFloor(floor_indicator,target,direction);
					printf("To floor 1\n");
					break;
				case TO_FLOOR_2:
					floor_indicator[1] = 1;
					if (!arrived) target = updateTargetFloor(floor_indicator,target,direction);
					printf("To floor 2\n");
					break;
				case TO_FLOOR_3:
					floor_indicator[2] = 1;
					if (!arrived) target = updateTargetFloor(floor_indicator,target,direction);
					printf("To floor 3\n");
					break;
				
				// ARRIVED_AT_FLOOR - if it's the target floor, start waiting
				case ARRIVED_AT_FLOOR:
					pos = getCarPosition();
					current_floor = GET_FLOOR(pos);
					// is it the floor we're going to?
					if (current_floor != target)
						break;
					// reset floor indicator and start idling
					floor_indicator[current_floor-1] = 0;
					arrived = 1;
					idle_time = 0;
					// if we're at top or bottom floors, change direction
					if (current_floor == TOP_FLOOR)
						direction = Down;
					else if (current_floor == BOTTOM_FLOOR)
						direction = Up;
					printf("Arrived at floor %d\n", current_floor);
					break;
					
				case LEFT_FLOOR:
					current_floor = 0;
					printf("Left floor.\n");
					break;
				case DOORS_CLOSED:
					doors_closed = 1;
					printf("Doors closed.\n");
					break;
				case DOORS_OPENING:
					doors_closed = 0;
					printf("Doors opened.\n");
					break;
				case STOP_PRESSED:
					setCarMotorStopped(1);
					printf("Stop pressed!\n");
					break;
				case STOP_RELEASED:
					setCarMotorStopped(0);
					printf("Stop released.\n");
					break;
				default:
					break;
			}
		}
		
		// if arrived at floor, need to wait
		if (arrived)
		{
			if (idle_time < IDLE_TIME)
				idle_time += POLL_TIME;
			else
			{
				floor_indicator[current_floor-1] = 0;		// make sure we don't try to go back to where we are
				// we've finished waiting, go to next floor (if there's a floor to go to)
				// of course, we can only go if the doors are closed
				if (doors_closed)
				{
					target = updateTargetFloor(floor_indicator, 0, direction);
					if (target)
					{
						// elevator might now have changed direction
						// target should not equal current_floor, if it is then there's a bug
						if (target < current_floor)
						{
							arrived = 0;
							direction = Down;
						}
						else if (target > current_floor)
						{
							arrived = 0;
							direction = Up;
						}
						else
							printf("Warning: bug found - target floor == current floor\n");
					}
				}
			}
		}
		// don't care about drift, since we reset timer whenever elevator arrives at floor
		// also, time we wait at a floor is not critical, a few extra milliseconds (or even half a second or so) is not a problem
		vTaskDelay(POLL_TIME);
	}
}

void setupPlanner(unsigned portBASE_TYPE uxPriority) {
  xTaskCreate(plannerTask, "planner", 100, NULL, uxPriority, NULL);
}
