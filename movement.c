#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <chprintf.h>
#include <usbcfg.h>

#include "ch.h"
#include "hal.h"


static float dist; 

#define MAX_ERROR		400
#define MIN_ERROR		140
#define MAX_SPEED 		350
#define TARGET 			100

static uint16_t oldDistance;


int16_t regulator(uint16_t distance)
{
	
	if(distance > MAX_ERROR || distance < TARGET)
	{
		return 0;
	}
	if((distance< MAX_ERROR ) && distance > MIN_ERROR)
	{
		return MAX_SPEED;
	}
	else
	{
		return (distance-TARGET+10)*7;
	}
}


void movement()
{
	

	if(getMusique() && isStraight()&&(getStraightCount()> 7 ))
	{
		uint16_t rotationSpeed;
		uint16_t distance = VL53L0X_get_dist_mm();
		if(oldDistance > distance)
		{
			if(getStraightSide)		//rotation a gauche
			{
				left_motor_set_speed(50);
				right_motor_set_speed(-50);
			}
			else					//roation a droite 
			{
				left_motor_set_speed(-50);
				right_motor_set_speed(50);
			}
		}
		oldDistance = distance;
		uint16_t speed = regulator(VL53L0X_get_dist_mm());
		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		//chprintf((BaseSequentialStream *) &SDU1,"distance en mm: %d \n",VL53L0X_get_dist_mm());
		return;
	}
	 if(getMusique())
	{
		right_motor_set_speed(getLeftRotationSpeed());
		left_motor_set_speed(getRightRotationSpeed());
		return;
	}
	 else
	 {
		 right_motor_set_speed(0);
		 left_motor_set_speed(0);
	 }
}

