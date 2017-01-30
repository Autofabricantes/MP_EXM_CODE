#ifndef MyoControl_h
#define MyoControl_h

#include <Arduino.h>

class MyoControl {
    public:
	    MyoControl();
        MyoControl(uint8_t emg_pin);
        void sampling(unsigned long sampleTime);
        void calibration();
        bool activation();
    private:
        void blinkLED(uint8_t ledPin, unsigned int repeat, unsigned int blinkTime);
        void meanCalc(unsigned int meanSamples);
        double movAv();
        void mvcCalc(unsigned int mvcSamples);
        uint8_t _emg_pin;
        volatile unsigned int emg;
        double emgMean, emgMvc;
        bool sampleOk, isActive;
        unsigned long prevTime = 0;
};

#endif
