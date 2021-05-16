#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <chprintf.h>
#include <usbcfg.h>

#include "ch.h"
#include "hal.h"


static float dist; 

#define MAX_ERROR			400
#define MIN_ERROR			140
#define MAX_SPEED 			350
#define TARGET 				100
#define TARGET_MARGE		10

#define REGULATOR_FACTOR	7

#define STRAIGHT_CONF_TIME	5

static uint16_t oldDistance;

/*  -----------------------------------------------------
*	regulator for the distance
*   -----------------------------------------------------
*/
int16_t regulator(uint16_t distance)
{
	
	if(distance < TARGET)
	{
		return 0;
	}
	if(distance > MIN_ERROR)
	{
		return MAX_SPEED;
	}
	else
	{
		return (distance-TARGET+TARGET_MARGE)*REGULATOR_FACTOR;
	}
}

/*  -----------------------------------------------------
*	fonction to move in direction of the source
*   -----------------------------------------------------
*/
void movement()
{

	//when aligned with the source
	if(getMusique() && isStraight()&&(getStraightCount()> STRAIGHT_CONF_TIME))
	{

		
		uint16_t speed = regulator(VL53L0X_get_dist_mm());
		if(speed == 0)
		{
			set_frontLed(TRUE);
		}
		else
		{
			set_frontLed(FALSE);
		}
		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		return;
	}

	//when music is on, do the rotation
	 if(getMusique())
	{
		set_frontLed(FALSE);
		right_motor_set_speed(getLeftRotationSpeed());
		left_motor_set_speed(getRightRotationSpeed());
		return;
	}

	//when music is off, dont move
	 else
	 {
	 	set_frontLed(FALSE);
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	 }
}

