#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H




typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

bool getMusique(void);
int16_t getLeftRotationSpeed(void);
int16_t getRightRotationSpeed(void);
uint16_t getStraightCount(void);
bool isStraight(void);


bool checkTime (uint16_t time, uint16_t timeRef);
void detect_pick(uint8_t id, uint32_t ampl);
void detectMusique(void);

void algoPosAmpl(float amplL, float amplF,float amplR, float amplB);

void processMean(void);
void addNewAmpl(void);
void updateMaxAmp(void);

void processAudioData(int16_t *data, uint16_t num_samples);



#endif /* AUDIO_PROCESSING_H */
