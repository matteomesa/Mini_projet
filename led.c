#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>
#include "ch.h"
#include "hal.h"

#define TIME_SLEEP_LED		30
#define COUNTER_LED_DOWN	10
#define LED_WA_SIZE			128

static bool ledPick;
static bool frontLed;

static uint8_t coutnerLedPick;

/*  -----------------------------------------------------
*	création thread Led
*   -----------------------------------------------------
*/

static THD_WORKING_AREA(waLed, LED_WA_SIZE);
static THD_FUNCTION(Led, arg) 
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
     while(1)
    {
    	
    	update_ledPick();
    	update_FrontLed();



    	chThdSleepMilliseconds(TIME_SLEEP_LED);


    }
}


/*  -----------------------------------------------------
*	function tu update the state of the LED
*   -----------------------------------------------------
*/

void update_ledPick()
{

	if(getMusique())
	{
		set_all_led(TRUE);

		if(ledPick)
		{
			set_all_led(FALSE);
			coutnerLedPick++;
		}
		if(coutnerLedPick > COUNTER_LED_DOWN)
		{
			ledPick = FALSE;
		}
	}
	else
	{
		set_all_led(FALSE);
	}
}

void update_FrontLed()
{
	if(!getMusique())
	{
		set_front_led(FALSE);
		return;
	}
	set_front_led(frontLed);
}



/*  -----------------------------------------------------
*	fonction to change the LED state
*   -----------------------------------------------------
*/

void set_ledPick()
{
	ledPick = TRUE;
	coutnerLedPick =0;
}

void set_frontLed(bool state)
{
	frontLed = state;
}

void set_all_led(bool state)
{
	set_led(LED1,state);
	set_led(LED3,state);
	set_led(LED5,state);
	set_led(LED7,state);
}


/*  -----------------------------------------------------
*	function to start the thread
*   -----------------------------------------------------
*/

void led_start(void){
	chThdCreateStatic(waLed, sizeof(waLed), NORMALPRIO, Led, NULL);
}
