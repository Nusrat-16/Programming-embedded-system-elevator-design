/**
 * Program skeleton for the course "Programming embedded systems"
 *
 * Lab 1: the elevator control system
 */

/**
 * Functions listening for changes of specified pins
 */

#include "FreeRTOS.h"
#include "task.h"

#include "pin_listener.h"
#include "assert.h"

#define STATUS_RELEASED		0		// button is not pressed
#define	STATUS_RELEASING	1		// button might be released
#define STATUS_PRESSING		2		// button might be pressed
#define STATUS_PRESSED		3		// button is pressed

static void pollPin(PinListener *listener,
                    xQueueHandle pinEventQueue) {

	u8 bit;
	bit = GPIO_ReadInputDataBit(listener->gpio, listener->pin);

	switch(listener->status)
	{
		// button is currently pressed
		case STATUS_PRESSED:
			if (!bit)
				listener->status = STATUS_RELEASING;	// maybe release
			break;
		// button might be pressed
		case STATUS_PRESSING:
			if (bit)
			{
				// ok, button really is pressed
				listener->status = STATUS_PRESSED;
				xQueueSend(pinEventQueue, &listener->risingEvent, portMAX_DELAY);
			}
			else
				// no, it was not a real press. If it was, we'll try again next call
				listener->status = STATUS_RELEASED;
			break;
		// button might be release
		case STATUS_RELEASING:
			if (bit)
				// no, button was not released. If it was, we'll try again next call
				listener->status = STATUS_PRESSED;
			else
			{
				// button was released, send falling event
				listener->status = STATUS_RELEASED;
				xQueueSend(pinEventQueue, &listener->fallingEvent, portMAX_DELAY);
			}
			break;
		// button is currently released
		case STATUS_RELEASED:
			if (bit)
				// button might be pressed, wait to see
				listener->status = STATUS_PRESSING;
			break;
			default:
				break;
	}
}

static void pollPinsTask(void *params) {
  PinListenerSet listeners = *((PinListenerSet*)params);
  portTickType xLastWakeTime;
  int i;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    for (i = 0; i < listeners.num; ++i)
	  pollPin(listeners.listeners + i, listeners.pinEventQueue);
    
	vTaskDelayUntil(&xLastWakeTime, listeners.pollingPeriod);
  }
}

void setupPinListeners(PinListenerSet *listenerSet) {
  portBASE_TYPE res;

  res = xTaskCreate(pollPinsTask, "pin polling",
                    100, (void*)listenerSet,
					listenerSet->uxPriority, NULL);
  assert(res == pdTRUE);
}
