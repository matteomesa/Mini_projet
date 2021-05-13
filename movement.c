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
#define MAX_SPEED 		320
#define TARGET 			100



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
		return (distance-TARGET)*8;
	}
}


void movement()
{

	if(isStraight()&&(getStraightCount()> 7 ))
	{
		uint16_t speed = regulator(VL53L0X_get_dist_mm());
		left_motor_set_speed(speed);
		right_motor_set_speed(speed);
		chprintf((BaseSequentialStream *) &SDU1,"distance en mm: %d \n",VL53L0X_get_dist_mm());
	}
	else
	{
		right_motor_set_speed(getLeftRotationSpeed());
		left_motor_set_speed(getRightRotationSpeed());
	}

	//detection musique
	//if( getMusique())

	//{
		//chprintf((BaseSequentialStream *) &SDU1,"Left rotation : %d \n",getLeftRotationSpeed());
		//chprintf((BaseSequentialStream *) &SDU1,"Right rotation : %d \n",getRightRotationSpeed());
		//right_motor_set_speed(getLeftRotationSpeed());
		//left_motor_set_speed(getRightRotationSpeed());
		//detection de position

		//detection de dist
			
		// PI de la distance 

		//mouvement
	//}
}
