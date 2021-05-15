#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>
//#include <led.h>
#include "ch.h"
#include "hal.h"


static bool ledPick;
static bool frontLed;

static uint8_t coutnerLedPick;


static THD_WORKING_AREA(waLed, 128);
static THD_FUNCTION(Led, arg) 
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
     while(1)
    {
    	
    	update_ledPick();
    	update_FrontLed();



    	chThdSleepMilliseconds(30);


    }
}

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
		if(coutnerLedPick > 10)
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


void led_start(void){
	chThdCreateStatic(waLed, sizeof(waLed), NORMALPRIO, Led, NULL);
}

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
