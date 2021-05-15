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
static uint8_t numberLed;

static THD_WORKING_AREA(waLed, 128);
static THD_FUNCTION(Led, arg) 
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
     while(1)
    {
    	set_front_led(getMusique());


    }
}

void update_ledPick()
{
	if(getMusique())	
	{
		set_led(LED1,TRUE);
		set_Led(LED3,TRUE);
		set_led(LED5,TRUE);
		set_Led(LED7,TRUE);
		if(ledPick && coutnerLedPick < 4)
		{
			//eteindre une led numberLed
			set_led(numberLed,FALSE);
			coutnerLedPick++;
		}
		else
		{
			//rallumer la led numberLed
			set_led(numberLed,TRUE);
			if(numberLed == 4)
			{
				numberLed = 0;
			}
			else
			{
				numberLed ++;
			}
			ledPick = FALSE;
		}	
	}	
}

void update_FrontLed()
{
	if(!getMusique())
	{
		set_front_led(FALSE);
	}
	if(frontLed)
	{
		set_front_led(frontLed);
	}
}
void led_start(void){
	chThdCreateStatic(waLed, sizeof(waLed), NORMALPRIO, Led, NULL);
}

void set_ledPick()
{
	ledPick = TRUE;
}

void set_frontLed(bool state)
{
	frontLed = state;
}