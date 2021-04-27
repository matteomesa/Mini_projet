#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

//Static float 
static float old_phase_diff;
static uint8_t nbFail;

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing
#define ALPHA			0.7 // coefficient to filter diff phase

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

float phase(float rea, float im)
{
	return atan2(im,rea);
}

float getPhase(float* FFTresult, float freq)
{
	uint16_t index = ceil(freq/15.625);
	return phase(FFTresult[2*index],FFTresult[2*index+1]);
	
}

float getPhaseMax(float* data,float* FFTresult)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	float phase_max = 0;
	uint16_t max_index = 0;


	for (uint16_t i=0; i < FFT_SIZE/2; i++)
	{
		if(data[i]>max_norm)
		{
			max_norm = data[i];
			max_index = i;
		}
	}

	//chprintf((BaseSequentialStream *) &SDU1, " indice phase = %d \n ",max_index);
	return phase(FFTresult[2*max_index],FFTresult[2*max_index+1]);

}

float getFreqMax(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	uint16_t max_index = 0;

	for (uint16_t i=0; i < FFT_SIZE/2; i++)
	{
		if(data[i]>max_norm)
		{
			max_norm = data[i];
			max_index = i;
		}
	}
	//chprintf((BaseSequentialStream *) &SDU1, " indice freq = %d \n ",max_index);

	return max_index * 15.625;
}

float diff_phase(float new_diff_phase)
{
	return (ALPHA*new_diff_phase + (1-ALPHA)*old_phase_diff);
}

void move(float error,float freq)
{
	//if(nbFail >10)
	//{
	//	left_motor_set_speed(0);
	//	right_motor_set_speed(0);
	//}


	if ((freq > 500)||(freq<300)||(abs(error) > 60))
	{
		//if(nbFail >10)
		//{
			//left_motor_set_speed(0);
			//right_motor_set_speed(0);
			return;
		//}
		//else
		//{
		//	nbFail ++;
		//	return;
		//}
	}

	if(abs(error) <= 10)
	{
		nbFail = 0;
		left_motor_set_speed(-error*20);
		right_motor_set_speed(error*20);
	}

	if( error < -10)
	{
		nbFail = 0;
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	else if(error > 10)
	{
		nbFail = 0;
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	else
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}

void movefreq(float* FFT_left, float* FFT_right,float freq,float* ampl)
{
	
	uint16_t index = ceil(freq/15.625);
	uint16_t maxindex = index;
	float maxampl = ampl[index];

	if (index < FFT_SIZE-1){
		if (ampl[index]<ampl[index+1]){
			maxampl = ampl[index+1];
			maxindex = index+1;
		}
	}
	if(index>0){
		if(ampl[index]<ampl[index-1]){
			maxampl = ampl[index-1];
			maxindex = index-1; 
		}
	}

	float phaseRight400 = getPhase(micRight_cmplx_input,maxindex);
	float phaseLeft400 = getPhase(micLeft_cmplx_input,maxindex);

	float error = diff_phase(phaseRight400 - phaseLeft400)*180/PI,freqMax;


	if(maxampl<4000)
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}



	if(abs(error)*20 <= 200)
	{
		nbFail = 0;
		left_motor_set_speed(-error*20);
		right_motor_set_speed(error*20);
	}

	if( error < -10)
	{
		nbFail = 0;
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	else if(error > 10)
	{
		nbFail = 0;
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	else
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}

void mess_ampl400(float ampl)
{
	if (ampl > 4000)
	{
		chprintf((BaseSequentialStream *) &SDU1, " -- ROTATION ON --");
	}
	else
	{
		chprintf((BaseSequentialStream *) &SDU1, " -- ROTATION OFF --");
	}
}
/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint16_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);


		float phaseRight = getPhaseMax(micRight_output,micRight_cmplx_input);
		float phaseLeft = getPhaseMax(micLeft_output,micLeft_cmplx_input);

		float difPhase =  diff_phase(phaseRight - phaseLeft);
		float freqMax = getFreqMax(micRight_output);

		move(difPhase*180/PI,freqMax);

		uint16_t index400 = ceil(400/15.625);
		float amplitude400 = micRight_output[index400];

		float phaseRight400 = getPhase(micRight_cmplx_input,400);
		float phaseLeft400 = getPhase(micLeft_cmplx_input,400);

		float difPhase400 =  diff_phase(phaseRight400 - phaseLeft400);

		chprintf((BaseSequentialStream *) &SDU1, "\n diff phase 400 = %1.4f, amplitude 400 = %1.4f ",difPhase400*180/PI,amplitude400);
		mess_ampl400(amplitude400);

		//move(difPhase400*180/PI,400,amplitude400);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			//chBSemSignal(&sendToComputer_sem);


			//chprintf((BaseSequentialStream *) &SDU1, "  amplitude 400 = %f \n",amplitude400);

			//chprintf((BaseSequentialStream *) &SDU1,"nbFail = %f \n",nbFail);
			
	
			//chprintf((BaseSequentialStream *) &SDU1, "  dif de phase = %f, freq max = %f \n",difPhase*180/PI,freqMax);


			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else{
		return NULL;
	}
}

