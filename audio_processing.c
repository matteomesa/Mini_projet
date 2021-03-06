#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>



#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <led.h>
#include <fft.h>
#include <arm_math.h>


#define FFT_SIZE		1024
#define NB_MEAN			5


#define NB_NOTE			4

#define TIME1			57350
#define TIME2 			14350 
#define TIME3			14350
#define TIME4			14350


#define NB_FREQ			3

#define FREQ_1			531 //first frequence to detect sound
#define FREQ_2			671 //second frequence to detect sound
#define FREQ_3			796 //third frequence to detect sound

#define FREQ_1_id		0
#define FREQ_2_id		1
#define FREQ_3_id		2

#define FREQ_1_POS		34
#define FREQ_2_POS		44
#define FREQ_3_POS		51

#define ROTATION_SPEED	250

#define PIC_FACTOR		4
#define PIC_TIM_LIM		7000
#define NB_OLD_PIC		2

#define NOISE 			5000
#define HIGH_NOISE		15000


#define NB_MIC			4

#define R_ID            0
#define L_ID            1
#define B_ID            2
#define F_ID            3

#define RANGE_TIME		6100
#define LIM_TIME		200
#define TIME_NO_MUSIC	25

#define RATIO_ROT		0.4

#define MAX_COUNTER		65000


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

//tableau static


static const uint8_t FREQ_ID_1024[NB_FREQ]={FREQ_1_POS,FREQ_2_POS,FREQ_3_POS};

//tableau pour stocker les temps entre nos picks
static uint16_t tabTime[NB_NOTE]; 
//tableau avec les temps de référence
static const uint16_t tabTimeRef[NB_NOTE]={TIME1,TIME2,TIME3,TIME4};
//tableau pour les dernières valeurs en amplitude max
static float tabPick[NB_OLD_PIC*NB_FREQ];

//tableau d'amplitude
static uint32_t lastAmpl[NB_FREQ][NB_MIC][NB_MEAN];
static uint32_t meanAmpl[NB_FREQ][NB_MIC];
static uint32_t tabMaxAmpl[NB_MIC];

static uint8_t idAmpl[NB_FREQ];


//counter
static uint16_t coutnerLastPick;
static uint16_t straight_count;
static uint8_t index_tab;

//vitesse de rotation
static int16_t leftRotationSpeed;
static int16_t rightRotationSpeed;

//bool d'état
static bool musique;
static bool straight;







/*  -----------------------------------------------------
*	Simple function used to return static to other thread
*   -----------------------------------------------------
*/

bool getMusique(void)
{
	return musique;
}

bool isStraight(void)
{
	return straight;
}


uint16_t getStraightCount(void)
{
	return straight_count;
}

int16_t getLeftRotationSpeed(void)
{
	return leftRotationSpeed;
}

int16_t getRightRotationSpeed(void)
{
	return rightRotationSpeed;
}


/*  -----------------------------------------------------
*	3 functions used to detect music
*   -----------------------------------------------------
*/

bool checkTime (uint16_t time, uint16_t timeRef)
{
	if(abs(time - timeRef) < LIM_TIME)
	{
		return TRUE;
	}
	else if(abs((time+RANGE_TIME) - timeRef) < LIM_TIME)
	{
		return TRUE;
	}
	else if(abs((time-RANGE_TIME) - timeRef) < LIM_TIME)
	{
		return TRUE;
	}
	else if(abs(time - 2*timeRef) < LIM_TIME)
	{
		return TRUE;
	}
	else if (abs(time - 2*timeRef - RANGE_TIME) < LIM_TIME)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


void detect_pick(uint8_t id, uint32_t ampl)
{
	if(ampl > NOISE)
	{
		float time = GPTD12.tim->CNT;
		if((ampl > PIC_FACTOR*tabPick[NB_OLD_PIC*id])&&(time>PIC_TIM_LIM))
		{
			set_ledPick();

			coutnerLastPick = 0;
			tabTime[index_tab] = time;
			GPTD12.tim->CNT = 0;
			
			index_tab++;

			if(index_tab == NB_NOTE)
			{
				index_tab =0;
			}
		}
		tabPick[0+NB_OLD_PIC*id] = tabPick[1+NB_OLD_PIC*id];
		tabPick[1+NB_OLD_PIC*id] = ampl;
	}
}


void detectMusique(void)
{

	//création double tableau de temps

	uint16_t tabTime2[2*NB_NOTE];

	for (uint8_t i=0;i<NB_NOTE;i++)
	{
		tabTime2[i] = tabTime[i];
	}
	for (uint8_t i=0;i<NB_NOTE;i++)
	{
		tabTime2[NB_NOTE+i] = tabTime[i];
	}

	bool temp_musique = false;

	//comparaison tableau

	for(uint8_t i = 0; i<NB_NOTE;i++)
	{
		if(checkTime(tabTime2[i],tabTimeRef[0])) 
		{
			
			bool error = true;
			for(uint8_t j = 1; j<NB_NOTE;j++)
			{
				
					
				if(checkTime(tabTime2[i+j],tabTimeRef[j])) 
				{
					
					if((j == NB_NOTE-1)&& error &&(coutnerLastPick < TIME_NO_MUSIC))
					{
						temp_musique = true;
					}
				}
				else
				{
					error = false;
				}

			}
		}

	}
	
	musique = temp_musique;
}

void algoPosAmpl(float amplL, float amplF,float amplR, float amplB)
{
	//Gauche 0-45
	if((amplL-amplR) > 0 && (amplF-amplB) > 0 && (amplL-amplF) < 0)
	{
		//not close
		if((amplL-amplR)/(amplF-amplL)>RATIO_ROT)
		{
			leftRotationSpeed  = ROTATION_SPEED;
			rightRotationSpeed =  -ROTATION_SPEED;

			straight = false;
			straight_count = 0;
			return;
		}
		//very close
		else
		{
			leftRotationSpeed  = 0;
			rightRotationSpeed = 0;

			straight = true;
			straight_count++;
			return;
		}		
	}
	//Gauche 45-90
	if((amplL-amplR) > 0 && (amplF-amplB) > 0 && (amplL-amplF) > 0)
	{
		leftRotationSpeed  =  ROTATION_SPEED;
		rightRotationSpeed = -ROTATION_SPEED;

		straight = false;
		straight_count = 0;
		return;
	}
	//Droite 0-45
	if((amplL-amplR) < 0 && (amplF-amplB) > 0 && (amplF-amplR) > 0)
	{
		//not close
		if((amplL-amplR)/(amplL-amplF)>RATIO_ROT)
		{
			leftRotationSpeed  =  -ROTATION_SPEED;
			rightRotationSpeed = ROTATION_SPEED;

			straight = false;
			straight_count = 0;
			return;
		}
		//close
		else
		{
			leftRotationSpeed  = 0;
			rightRotationSpeed = 0;

			straight = true;
			straight_count++;
			return;
		}
	}
	//Droite 45-90
	if((amplL-amplR) < 0 && (amplF-amplB) > 0 && (amplF-amplR) < 0)
	{
		leftRotationSpeed  =  -ROTATION_SPEED;
		rightRotationSpeed = ROTATION_SPEED;

		straight = false;
		straight_count = 0;
		return;
	}
	//Droite 90-135
	if((amplL-amplR) < 0 && (amplF-amplB) < 0 && (amplR-amplB) > 0)
	{
		leftRotationSpeed  =  -ROTATION_SPEED;
		rightRotationSpeed = ROTATION_SPEED;

		straight = false;
		straight_count = 0;
		return;
	}
	//Droite 135-180
	if((amplL-amplR) < 0 && (amplF-amplB) < 0 && (amplR-amplB) < 0)
	{
		leftRotationSpeed  =  -ROTATION_SPEED;
		rightRotationSpeed = ROTATION_SPEED;

		straight = false;
		straight_count = 0;
		return;
	}
	//Gauche 135-180
	if((amplL-amplR) > 0 && (amplF-amplB) < 0 && (amplB-amplL) > 0)
	{
		leftRotationSpeed  = ROTATION_SPEED;
		rightRotationSpeed =  -ROTATION_SPEED;

		straight = false;
		straight_count = 0;
		return;
	}
	//Gauche 90-135
	if((amplL-amplR) > 0 && (amplF-amplB) < 0 && (amplB-amplL) < 0)
	{
		leftRotationSpeed  = ROTATION_SPEED;
		rightRotationSpeed =  -ROTATION_SPEED;

		straight = false;
		straight_count = 0;
		return;
	}

}


void processMean(void)
{
	for (uint8_t i = 0; i<NB_FREQ;i++)
	{

		for(uint8_t j = 0; j<NB_MIC;j++)
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

void addNewAmpl(void)
{
	for (uint8_t i = 0; i<NB_FREQ;i++)
	{
		if(micRight_output[FREQ_ID_1024[i]] > HIGH_NOISE)
		{

			lastAmpl[i][R_ID][idAmpl[i]] = (uint32_t)micRight_output[FREQ_ID_1024[i]];
			lastAmpl[i][L_ID][idAmpl[i]] = (uint32_t)micLeft_output[FREQ_ID_1024[i]];
			lastAmpl[i][F_ID][idAmpl[i]] = (uint32_t)micFront_output[FREQ_ID_1024[i]];
			lastAmpl[i][B_ID][idAmpl[i]] = (uint32_t)micBack_output[FREQ_ID_1024[i]];

			idAmpl[i]++;
			if(idAmpl[i] >= NB_MEAN)
			{
				idAmpl[i] = 0;
			}
		}
	}
}


void updateMaxAmp(void)
{
	tabMaxAmpl[0] = 0;
	tabMaxAmpl[1] = 0;
	tabMaxAmpl[2] = 0;
	tabMaxAmpl[3] = 0;
	for (uint8_t i =0; i<NB_FREQ;i++)
		{
			if(tabMaxAmpl[0] < meanAmpl[i][R_ID])
			{
				tabMaxAmpl[0] = meanAmpl[i][R_ID];
				tabMaxAmpl[1] = meanAmpl[i][L_ID];
				tabMaxAmpl[2] = meanAmpl[i][B_ID];
				tabMaxAmpl[3] = meanAmpl[i][F_ID];
			}
		}
}

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

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

		// modification de tabMaxAmpl avec les nouvelles valeurs 

		processMean();
		addNewAmpl();
		updateMaxAmp();

		//modification vitesse rotation

		algoPosAmpl(tabMaxAmpl[L_ID],tabMaxAmpl[F_ID],tabMaxAmpl[R_ID],tabMaxAmpl[B_ID]);

		//detection musique

		detect_pick(FREQ_1_id, micRight_output[FREQ_ID_1024[FREQ_1_id]]);
		detect_pick(FREQ_2_id, micRight_output[FREQ_ID_1024[FREQ_2_id]]);
		detect_pick(FREQ_3_id, micRight_output[FREQ_ID_1024[FREQ_3_id]]);

		detectMusique();

		nb_samples = 0;
		if(coutnerLastPick < MAX_COUNTER)
		{
			coutnerLastPick++;
		}
	}
}



