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
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];

static float micBack_output[FFT_SIZE];

static float tabFreq[4];
static uint16_t tabTime[4];
static float tabPick[6];

//Static float 
static float old_phase_diff;
static float OldFreq;

//Static int
static uint8_t nbFail;
static uint8_t counter;
static uint8_t index_tab;

static float rightDifPhase;
static float leftDifPhase;


//Static bool
static bool chrono;
static bool indexPick;


//Test degueu
static float lastAmplR[NB_MEAN];
static float lastAmplL[NB_MEAN];
static float lastAmplF[NB_MEAN];
static float lastAmplB[NB_MEAN];


//tableau des dernière amplitudes (3 freq / 4 mic / 10 donnée)
static float lastAmpl[4][4][NB_MEAN];

//moyenne des dernières mesures des mic
static float meanAmpl[4][4];

static uint8_t idAmpl[4];

static const uint8_t FREQ_ID_1024[4]={34,43,51,20};

static uint16_t leftRotationSpeed;
static uint16_t RightRotationSpeed;



static float lastdifPhase[NB_MEAN];



#define MIN_FREQ		20	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		60	//we don't analyze after this index to not use resources for nothing
#define ALPHA			0.7 // coefficient to filter diff phase
#define FREQ_1			531 //first frequence to detect sound
#define FREQ_2			671 //second frequence to detect sound
#define FREQ_3			796 //third frequence to detect sound
#define FREQ_4          312 //4ème fre

#define FREQ_1_id		0
#define FREQ_2_id		1
#define FREQ_3_id		2
#define FREQ_4_id		3

#define ROTATION_SPEED	300



#define NOISE 			5000
#define TIME_1			1
#define TIME_2			1
#define TIME_3			1
#define TIME_4			1

#define LIM_1			1
#define LIM_2			1
#define LIM_3			1
#define LIM_4			1
#define R_ID            0
#define L_ID            1
#define B_ID            2
#define F_ID            3

#define RANGE_TIME		6100





/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

bool checkTime (float time, float timeRef)
{
	if((time - timeRef) < 100)
	{
		return TRUE;
	}
	else if(((time+RANGE_TIME) - timeRef) < 100)
	{
		return TRUE;
	}
	else if((((time-RANGE_TIME) - timeRef) < 100))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool almostEgalLim(float a,float b,float lim)
{
	if((a-b)*(a-b)<lim*lim)
		return TRUE;
	else
		return FALSE;
}
float phase(float rea, float im)
{
	return atan2(im,rea);
}

float getPhase(float* FFTresult, float freq)
{
	uint16_t index = ceil(freq/15.625);
	return atan2(FFTresult[2*index],FFTresult[2*index+1]);
	
}

float getPhaseMax(float* data,float* FFTresult)
{
	float max_norm = 0;
	float phase_max = 0;
	uint16_t max_index = 0;


	for (uint16_t i=MIN_FREQ; i < MAX_FREQ; i++)
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
	float max_norm = 0;
	uint16_t max_index = 0;

	for (uint16_t i=MIN_FREQ; i < MAX_FREQ; i++)
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

float getAmplMax(float* data)
{
	float max_norm = 0;
	uint16_t max_index = 0;

	for (uint16_t i=MIN_FREQ; i < MAX_FREQ; i++)
	{
		if(data[i]>max_norm)
		{
			max_norm = data[i];
			max_index = i;
		}
	}
	return data[max_index];
}

float diff_phase(float new_diff_phase)
{
	float temp_phase = (ALPHA*new_diff_phase + (1-ALPHA)*old_phase_diff);
	//old_phase_diff = temp_phase;
	return temp_phase;
}

void detect_pick(uint8_t id, float ampl)
{
	if(ampl > NOISE)
	{
		float time = GPTD12.tim->CNT;
		if((ampl > 4*tabPick[0+2*id])&&(time>7000))
		{
			chprintf((BaseSequentialStream *) &SDU1,"pic detect, id = %d",id);
			tabTime[index_tab] = time;
			GPTD12.tim->CNT = 0;
			
			
			chprintf((BaseSequentialStream *) &SDU1," time = %f \n",time);
			if(index_tab == 3)
			{
				index_tab = 0;
			}
			else
			{
				index_tab++;
			}
			
			
		}
		tabPick[0+2*id] = tabPick[1+2*id];
		tabPick[1+2*id] = ampl;
	}
}
void algoPosAmpl(float amplL, float amplF,float amplR, float amplB)
{
	if((amplL-amplR) > 0 && (amplF-amplB) > 0 && (amplL-amplF) < 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 0-45, ratio = %1.4f \n",(amplL-amplR)/(amplF-amplR));

		if((amplL-amplR)/(amplF-amplR)>0.4)
		{
			left_motor_set_speed(-ROTATION_SPEED);
			right_motor_set_speed(ROTATION_SPEED);
		}
		else
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);

		}

		
	}
	if((amplL-amplR) > 0 && (amplF-amplB) > 0 && (amplL-amplF) > 0)
	{
		//rotation vers la gauche vitesse 2
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 45-90 \n");
		left_motor_set_speed(-ROTATION_SPEED);
		right_motor_set_speed(ROTATION_SPEED);
	}
	if((amplL-amplR) < 0 && (amplF-amplB) > 0 && (amplF-amplR) > 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 0-45, ratio = %1.4f \n",(amplL-amplR)/(amplL-amplF));

		if((amplL-amplR)/(amplF-amplL)>0.4)
		{
			left_motor_set_speed(ROTATION_SPEED);
			right_motor_set_speed(-ROTATION_SPEED);
		}
		else
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);

		}
	}
	if((amplL-amplR) < 0 && (amplF-amplB) > 0 && (amplF-amplR) < 0)
	{
		//rotation vers la droite vitesse 2
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 45-90 \n");
		left_motor_set_speed(ROTATION_SPEED);
		right_motor_set_speed(-ROTATION_SPEED);
	}
	if((amplL-amplR) < 0 && (amplF-amplB) < 0 && (amplR-amplB) > 0)
	{
		// rotation vers la droite vitesse 3 ou quart de tour vers la droite + rotation vers la droite vitesse 1
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 90-135 \n");
		left_motor_set_speed(ROTATION_SPEED);
		right_motor_set_speed(-ROTATION_SPEED);
	}
	if((amplL-amplR) < 0 && (amplF-amplB) < 0 && (amplR-amplB) < 0)
	{
		// rotation vers la droite vitesse 4 ou quart de tour vers la droite + rotation vers la droite vitesse 2
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 135-180 \n");
		left_motor_set_speed(ROTATION_SPEED);
		right_motor_set_speed(-ROTATION_SPEED);
	}
	if((amplL-amplR) > 0 && (amplF-amplB) < 0 && (amplB-amplL) > 0)
	{
		// rotation vers la gauche vitesse 3 ou quart de tour vers la gauche + rotation vers la gauche vitesse 1
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 90-135 \n");
		left_motor_set_speed(-ROTATION_SPEED);
		right_motor_set_speed(ROTATION_SPEED);
	}
	if((amplL-amplR) > 0 && (amplF-amplB) < 0 && (amplB-amplL) < 0)
	{
		// rotation vers la gauche vitesse 4 ou quart de tour vers la gauche + rotation vers la gauche vitesse 2
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 135-180 \n");
		left_motor_set_speed(-ROTATION_SPEED);
		right_motor_set_speed(ROTATION_SPEED);

	}

}
bool check_tab( uint16_t tab[4])
{
	
	for(uint8_t i = 0; i < 4 ;i++)
	{
		bool checkTab;
		switch(i)
		{
			case 0:
			{
				if(almostEgalLim(tab[i],TIME_1,LIM_1) && almostEgalLim(tab[i+1],TIME_2,LIM_3) && almostEgalLim(tab[i+2],TIME_3,LIM_3) && almostEgalLim(tab[i+2],TIME_4,LIM_4))
				{
					return TRUE;
				}
				else 
				{
					break;
				}
			}
			case 1:
			{
				if(almostEgalLim(tab[i],TIME_1,LIM_1) && almostEgalLim(tab[i+1],TIME_2,LIM_3) && almostEgalLim(tab[i+2],TIME_3,LIM_3) && almostEgalLim(tab[i-1],TIME_4,LIM_4))
				{
					return TRUE;
				}
				else
				{
					break;
				}		
			}
			case 2:
			{
				if(almostEgalLim(tab[i],TIME_1,LIM_1) && almostEgalLim(tab[i+1],TIME_2,LIM_3) && almostEgalLim(tab[i-2],TIME_3,LIM_3) && almostEgalLim(tab[i-1],TIME_4,LIM_4))
				{
					return TRUE;
				}
				else
				{
					break;
				}		
			}
			case 3:
			{
				if(almostEgalLim(tab[i],TIME_1,LIM_1) && almostEgalLim(tab[i-3],TIME_2,LIM_3) && almostEgalLim(tab[i-2],TIME_3,LIM_3) && almostEgalLim(tab[i-1],TIME_4,LIM_4))
				{
					return TRUE;
				}
				else	
				{
					return FALSE;
				}		
			}
		}
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


void processMean()
{
	for (uint8_t i = 0; i<4;i++)
	{

		for(uint8_t j = 0; j<4;j++)
		{
			meanAmpl[i][j] = 0;
			
			for(uint8_t k = 0; k<NB_MEAN;k++)
			{
				meanAmpl[i][j] += lastAmpl[i][j][k];

			}
			meanAmpl[i][j] = meanAmpl[i][j]/NB_MEAN;
		}

	}
}

void addNewAmpl()
{
	for (uint8_t i = 0; i<4;i++)
	{
		if(micRight_output[FREQ_ID_1024[i]] > 15000)
		{

			lastAmpl[i][R_ID][idAmpl[i]] = micRight_output[FREQ_ID_1024[i]];
			lastAmpl[i][L_ID][idAmpl[i]] = micLeft_output[FREQ_ID_1024[i]];
			lastAmpl[i][F_ID][idAmpl[i]] = micFront_output[FREQ_ID_1024[i]];
			lastAmpl[i][B_ID][idAmpl[i]] = micBack_output[FREQ_ID_1024[i]];

			idAmpl[i]++;
			if(idAmpl[i] >= NB_MEAN)
			{
				idAmpl[i] = 0;
			}
		}
	}
}



float getRightDifPhase()
{return rightDifPhase;}

float getLeftDifPhase()
{return leftDifPhase;}



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
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

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
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);


		float phaseRight = getPhaseMax(micRight_output,micRight_cmplx_input);
		float phaseLeft = getPhaseMax(micLeft_output,micLeft_cmplx_input);

		float difPhase =  diff_phase(phaseRight - phaseLeft);
		old_phase_diff = difPhase;
		float freqMax = getFreqMax(micRight_output);
		float amplMax = getAmplMax(micRight_output);

		float phaseRight535 = phase(micRight_output[2*34],micRight_output[2*34+1]);
		float phaseLeft535 = phase(micLeft_output[2*34],micLeft_output[2*34+1]);

		float difPhase535 = phaseRight535-phaseLeft535;



		float Ampl312 = micRight_output[20];
		float Ampl535 = micRight_output[34];
		float Ampl671 = micRight_output[43];
		float Ampl796 = micRight_output[51];

		processMean();
		addNewAmpl();

		float maxAmplRight =0;
		float maxAmplLeft =0;
		float maxAmplBack =0;
		float maxAmplFront =0;

		for (uint8_t i =0; i<4;i++)
		{
			if(maxAmplRight < meanAmpl[i][R_ID])
			{
				maxAmplRight = meanAmpl[i][R_ID];
				maxAmplLeft = meanAmpl[i][L_ID];
				maxAmplBack = meanAmpl[i][B_ID];
				maxAmplFront = meanAmpl[i][F_ID];
			}
		}


		//algoPosAmpl(maxAmplLeft,maxAmplFront,maxAmplRight,maxAmplBack);


		float ALmR = maxAmplLeft-maxAmplRight;
		float AFmR = maxAmplFront-maxAmplRight;

		float ratio = (ALmR/AFmR);

		//chprintf((BaseSequentialStream *) &SDU1,"%f %f %fa",ALmR,AFmR,ratio);


		detect_pick(FREQ_1_id, micRight_output[FREQ_ID_1024[FREQ_1_id]]);
		detect_pick(FREQ_2_id, micRight_output[FREQ_ID_1024[FREQ_2_id]]);
		detect_pick(FREQ_3_id, micRight_output[FREQ_ID_1024[FREQ_3_id]]);

//		detect_pick(0, Ampl535);
//		detect_pick(1, Ampl671);
//		detect_pick(2, Ampl796);







		//algoPosAmpl(meanAmpl535L,meanAmpl535F,meanAmpl535R,meanAmpl535B);


		//chprintf((BaseSequentialStream *) &SDU1,"%f %f %f %fa",Ampl312,Ampl535,Ampl671,Ampl796);


//		float difAmpl535RL = Ampl535R - Ampl535L;
//		float difAmpl535FB = Ampl535F - Ampl535B;
//
//
//		float difAmpl535 = Ampl535 - Ampl535L;
//
//		//chprintf((BaseSequentialStream *) &SDU1,"%fa",difPhase);
//
//		if (((abs(difPhase)*100>300)||(amplMax<10000))&&(idAmpl!=0))
//		{
//			lastdifPhase[idAmpl] = lastdifPhase[idAmpl-1];
//
//		}
//		if (((abs(difPhase)*100>70)||(amplMax<10000))&&(idAmpl==0))
//		{
//			lastdifPhase[idAmpl] = lastdifPhase[idAmpl+9];
//
//		}
//
//
//		lastAmplRL[idAmpl] = difAmpl535RL;
//		lastAmplFB[idAmpl] = difAmpl535FB;
//		lastdifPhase[idAmpl]=difPhase;
//
//		float meanDifPhase = 0;
//		for (uint8_t i = 0;i<10;i++)
//		{
//			meanDifPhase += lastdifPhase[i];
//
//		}
//			meanDifPhase = meanDifPhase/10;
//			//chprintf((BaseSequentialStream *) &SDU1,"%fa",meanDifPhase*100);
//
//		idAmpl++;
//		if (idAmpl ==10)
//		{
//			float meanAmplRL = 0;
//			float meanAmplFB = 0;
//			for (uint8_t i = 0;i<10;i++)
//			{
//				meanAmplRL += lastAmplRL[i];
//				meanAmplFB += lastAmplFB[i];
//
//			}
//			meanAmplRL = meanAmplRL/10;
//			meanAmplFB = meanAmplFB/10;
//			//chprintf((BaseSequentialStream *) &SDU1," dif Ampl = %f \n",meanAmpl);
//			idAmpl = 0;
//
//		}
//
//
//
//
//		detect_pick(0, Ampl535);
//		detect_pick(1, Ampl671);
//		detect_pick(2, Ampl796);
//
//		rightDifPhase = diff_phase(phaseRight - phaseLeft);
//		leftDifPhase = diff_phase(phaseLeft - phaseRight);
//
//
//
//		float diffPhase535 = phase(micRight_cmplx_input[2*34],micRight_cmplx_input[2*34+1])*100;
//
//		float diffPhase671 = phase(micRight_cmplx_input[2*43],micRight_cmplx_input[2*43+1])*100;
//
//		float diffPhase796 = phase(micRight_cmplx_input[2*51],micRight_cmplx_input[2*51+1])*100;
//		float tempDiff = 0;
//		float diffPhase =  (diffPhase535 + diffPhase671 + diffPhase796)*0.33;
//		//chprintf((BaseSequentialStream *) &SDU1,"%1.1f %1.1f %1.1f %1.1fa", diffPhase535, diffPhase671, diffPhase796,diffPhase );
//		if(Ampl535 > 10000 || Ampl671 > 10000 || Ampl796 > 10000)
//		{
//			float diffPhase =  (diffPhase535 + diffPhase671 + diffPhase796)*0.33;
//		//	chprintf((BaseSequentialStream *) &SDU1, " %1.1fa", diffPhase);
//			tempDiff = diffPhase;
//		}
//		else
//		{
//
//		//	chprintf((BaseSequentialStream *) &SDU1, " %1.1fa", tempDiff);
//		}
//
//
//		if((Ampl535 > 10000)&&(abs(difPhase535)<1))
//		{
//			chprintf((BaseSequentialStream *) &SDU1, "%f %fa", difPhase535,Ampl535);
//
//		}

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

