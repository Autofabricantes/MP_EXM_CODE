#include "MyoControl.h"

static const unsigned int sampleTime = 1;
static const double adcRef = 5.00;
static const unsigned int adcRes = 1023;
static const double adcConv = adcRef/adcRes;

MyoControl::MyoControl(){
}

MyoControl::MyoControl(uint8_t emg_pin) {

	logger.debug("MyoControl - MyoControl\n");

    pinMode(emg_pin, INPUT);
    emg_pin = _emg_pin;
}

/* blinkLED blinks a led "repeat" times with a "bTime" interval between on and off */
void MyoControl::blinkLED(uint8_t ledPin, unsigned int repeat, unsigned int bTime) {

	logger.debug("MyoControl - blinkLED\n");

    pinMode(ledPin, OUTPUT);
    unsigned int i;
    for(i=0;i<repeat;i++) {
        digitalWrite(ledPin,HIGH);
        delay(bTime);
        digitalWrite(ledPin,LOW);
        delay(bTime);
    }
}

/* sampling reads the ADC every 1 ms with the MsTimer2 interrupt. */
void MyoControl::sampling() {

    emg = analogRead(_emg_pin);
    sampleOk = 1; // sampleOk indicates that a new sample is ready to be processed
	//logger.debug("MyoControl - sampling - sampleOK\n");
}

/* meanCalc computes the mean value of the EMG signal during a period of
meanSamples. The mean value of the signal is its baseline. This value is
subtracted from the measured EMG signals to have a baseline of 0 V. */
void MyoControl::meanCalc(unsigned int meanSamples)
{
	logger.debug("MyoControl - meanCalc\n");

    unsigned int i = 0;
    while(i < meanSamples)
    {
        delayMicroseconds(50);
        if(sampleOk==1)
        {
			logger.debug("MyoControl - meanCalc - sampleOK\n");
			sampleOk = 0;
            i++;
            emgMean = emgMean + emg*adcConv;
        }

		logger.debug("MyoControl - meanCalc - i: %d\n", i);
		logger.debug("MyoControl - meanCalc - meanSamples: %d\n", meanSamples);


    }
    i = 0;
    emgMean = emgMean/meanSamples;
}

/* movAv computes the running moving average of the EMG signal. First, the
baseline of the signal is lowered to 0 V, to be able to rectify it. The running
moving average rectifies and smooths the signal (acts as a low pass filter),
returning the amplitude of the signal. */
double MyoControl::movAv() {

    double emgZero = 0.00, emgMovav = 0.00;
    emgZero = emg*adcConv - emgMean; // Signal with 0 V baseline
    emgMovav = emgMovav*0.99 + abs(emgZero)*0.01; // Rectified and smoothed signal
    return emgMovav;
}

/* mvcCalc computes the maximum voluntary contraction, the maximum force the
user is able to exert. This value is used to compute the activation threshold */
void MyoControl::mvcCalc(unsigned int mvcSamples) {

	logger.debug("MyoControl - mvcCalc\n");

    double emgMovav = 0.00;
    unsigned int i = 0;
    while(i < mvcSamples) {
        delayMicroseconds(50);
        if(sampleOk==1) {
            sampleOk = 0;
            i++;
            emgMovav = movAv();
            if(emgMovav > emgMvc) {
                emgMvc = emgMovav;
            }
        }

		logger.debug("MyoControl - mvcCalc - i: %d\n", i);
		logger.debug("MyoControl - mvcCalc - mvcSamples: %d\n", mvcSamples);
    }
}

void MyoControl::calibration() {

	logger.debug("MyoControl - calibration\n");

    /* System calibration */
    /* Calibration step #1: calculate the baseline of the signal during 10 s */
    blinkLED(13,1,500); // LED blinks once to indicate calibration step #1 start
    meanCalc(10000);
    blinkLED(13,1,500); // LED blinks once to indicate calibration step #1 end
    delay(1000);
    /* Calibration step #2: calculate the maximum voluntary contraction during 5 s*/
    blinkLED(13,2,500); // LED blinks twice to indicate calibration step #2 start
    mvcCalc(5000);
    blinkLED(13,2,500); // LED bliks twice to indicate calibration step #2 end
    delay(1000);
}

bool MyoControl::activation() {

	logger.debug("MyoControl - activation\n");
	//pinMode(13,OUTPUT);
    delayMicroseconds(50);
    if(sampleOk) {
        sampleOk = 0;
        double emgMovav = movAv(); // Gets the amplitude of the measured EMG signal

        /* If the amplitude of the EMG signal is greater than the activation threshold
        (a 35% of the MVC), there is a muscle activation. */
        if(emgMovav > 0.35*emgMvc) {
	    //digitalWrite(13,HIGH);
            return true;
        }
        /* If the amplitude of the EMG signal is below the activation threshold,
        there is no muscle activation. */
        else {
	    //digitalWrite(13, LOW);
            return false;
        }
    }
    else {
        return false;
    }
}
