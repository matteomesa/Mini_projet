#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <arm_math.h>


static float dist; 

#define MAX_SUM_ERROR		100
#define ERROR_THRESHOLD		10
#define KP 					1
#define KI					1

int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

void movement()
{
	//detection musique
	if( getMusique())
	{
		right_motor_set_speed(getLeftRotationSpeed());
		left_motor_set_speed(getRightRotationSpeed());
		//detection de position

		//detection de dist
			
		// PI de la distance 

		//mouvement
	}
}