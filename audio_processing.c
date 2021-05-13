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

static uint8_t tabFreq[4];
static uint16_t tabTime[4];

static float tabPick[6];

//Static float 


//Static int
static uint8_t index_tab;


//Static bool

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

static float tabMaxAmpl[4];

static const uint8_t FREQ_ID_1024[4]={34,43,51,20};

static const uint8_t tabFreqRef[4]={0,0,1,2};

static const uint16_t tabTimeRef[4]={57350,14350,14350,14350};

static uint16_t leftRotationSpeed;
static uint16_t rightRotationSpeed;



static float lastdifPhase[NB_MEAN];

static bool musique;



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

#define R_ID            0
#define L_ID            1
#define B_ID            2
#define F_ID            3

#define RANGE_TIME		6100

#define LIM_TIME		200

#define RATIO_ROT	0.4



/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/


bool getMusique()
{
	return musique;
}

uint8_t getLeftRotationSpeed()
{
	return leftRotationSpeed;
}

uint8_t getRightRotationSpeed()
{
	return rightRotationSpeed;
}


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

void detect_pick(uint8_t id, float ampl)
{
	if(ampl > NOISE)
	{
		float time = GPTD12.tim->CNT;
		if((ampl > 4*tabPick[0+2*id])&&(time>7000))
		{
			//chprintf((BaseSequentialStream *) &SDU1,"pic detect, id = %d",id);
			tabTime[index_tab] = time;
			tabFreq[index_tab] = id;
			GPTD12.tim->CNT = 0;
			
			
			//chprintf((BaseSequentialStream *) &SDU1," time = %f \n",time);
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


void detectMusique()
{
	uint16_t tabTime2[8];
	uint8_t tabFreq2[8];

	for (uint8_t i=0;i<4;i++)
	{
		tabTime2[i] = tabTime[i];
		tabFreq2[i] = tabFreq[i];
	}
	for (uint8_t i=0;i<4;i++)
	{
		tabTime2[4+i] = tabTime[i];
		tabFreq2[4+i] = tabFreq[i];
	}

	bool temp_musique = false;

	//comparaison tableau

	for(uint8_t i = 0; i<4;i++)
	{
		if( (tabFreq2[i]==tabFreqRef[0]) && (checkTime(tabTime2[i],tabTimeRef[0])) )
		{
			
			bool error = true;
			for(uint8_t j = 1; j<4;j++)
			{
				
					
				if( (tabFreq2[i+j]==tabFreqRef[j]) && (checkTime(tabTime2[i+j],tabTimeRef[j])) )
				{
					
					if((j==3)&& error)
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
	set_body_led(musique);


}

void algoPosAmpl(float amplL, float amplF,float amplR, float amplB)
{
	if((amplL-amplR) > 0 && (amplF-amplB) > 0 && (amplL-amplF) < 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 0-45, ratio = %1.4f \n",(amplL-amplR)/(amplF-amplR));

		{
			leftRotationSpeed  = -ROTATION_SPEED;
			rightRotationSpeed =  ROTATION_SPEED;
		}
		else
		{
			leftRotationSpeed  = 0;
			rightRotationSpeed = 0;
		}		
	}
	if((amplL-amplR) > 0 && (amplF-amplB) > 0 && (amplL-amplF) > 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 45-90 \n");
		leftRotationSpeed  = -ROTATION_SPEED;
		rightRotationSpeed =  ROTATION_SPEED;
	}
	if((amplL-amplR) < 0 && (amplF-amplB) > 0 && (amplF-amplR) > 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 0-45, ratio = %1.4f \n",(amplL-amplR)/(amplL-amplF));

		if((amplL-amplR)/(amplF-amplL)>RATIO_ROT)
		{
			leftRotationSpeed  =  ROTATION_SPEED;
			rightRotationSpeed = -ROTATION_SPEED;
		}
		else
		{
			leftRotationSpeed  = 0;
			rightRotationSpeed = 0;
		}
	}
	if((amplL-amplR) < 0 && (amplF-amplB) > 0 && (amplF-amplR) < 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 45-90 \n");
		leftRotationSpeed  =  ROTATION_SPEED;
		rightRotationSpeed = -ROTATION_SPEED;
	}
	if((amplL-amplR) < 0 && (amplF-amplB) < 0 && (amplR-amplB) > 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 90-135 \n")
		leftRotationSpeed  =  ROTATION_SPEED;
		rightRotationSpeed = -ROTATION_SPEED;
	}
	if((amplL-amplR) < 0 && (amplF-amplB) < 0 && (amplR-amplB) < 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Droite 135-180 \n");
		leftRotationSpeed  =  ROTATION_SPEED;
		rightRotationSpeed = -ROTATION_SPEED;
	}
	if((amplL-amplR) > 0 && (amplF-amplB) < 0 && (amplB-amplL) > 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 90-135 \n");
		leftRotationSpeed  = -ROTATION_SPEED;
		rightRotationSpeed =  ROTATION_SPEED;
	}
	if((amplL-amplR) > 0 && (amplF-amplB) < 0 && (amplB-amplL) < 0)
	{
		//chprintf((BaseSequentialStream *) &SDU1,"Gauche 135-180 \n");
		leftRotationSpeed  = -ROTATION_SPEED;
		rightRotationSpeed =  ROTATION_SPEED;
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


{
	tabMaxAmpl[0] = 0;
	tabMaxAmpl[1] = 0;
	tabMaxAmpl[2] = 0;
	tabMaxAmpl[3] = 0;
	
	for (uint8_t i =0; i<4;i++)
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

		processMean();
		addNewAmpl();
		updateMaxAmp();

		

		//algoPosAmpl(maxAmplLeft,maxAmplFront,maxAmplRight,maxAmplBack);
		//float ALmR = maxAmplLeft-maxAmplRight;
		//float AFmR = maxAmplFront-maxAmplRight;

		//float ratio = (ALmR/AFmR);
		detect_pick(FREQ_1_id, micRight_output[FREQ_ID_1024[FREQ_1_id]]);
		detect_pick(FREQ_2_id, micRight_output[FREQ_ID_1024[FREQ_2_id]]);
		detect_pick(FREQ_3_id, micRight_output[FREQ_ID_1024[FREQ_3_id]]);


		float difAmpl1 = tabMaxAmpl[R_ID]-tabMaxAmpl[L_ID];
		float difAmpl2 = tabMaxAmpl[L_ID]-tabMaxAmpl[F_ID];
		float ratio = difAmpl1/difAmpl2;




		chprintf((BaseSequentialStream *) &SDU1,"%f %f %f a",difAmpl1,difAmpl2,ratio);




		//detectMusique();

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

