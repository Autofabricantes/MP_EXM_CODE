#include "Test.h"

/******************************************************************************/
/* PUBLIC METHODS                                                             */
/******************************************************************************/

int Test::testInputForTransition(){

	int transition = 0;
	
	logger.info("Trans to: \n");
	logger.info(" (0) STATE_INACTIVE\n");
	logger.info(" (1) STATE_IDLE\n");
	logger.info(" (2) STATE_TONGS\n");
	logger.info(" (3) STATE_FINGER\n");
	logger.info(" (4) STATE_CLOSE\n");
	logger.info(" (5) STATE_FIST\n");

	// send data only when you receive data:
	while(!Serial.available());

	// read the incoming byte:
	transition = Serial.parseInt();

	return transition;
  
}

void Test::testInitializeLedStripe(){


	ledStripe = Adafruit_NeoPixel(10, PIN_OUTPUT_LEDSTRIPE_RGB, NEO_GRB + NEO_KHZ800);
	ledStripe.begin();

	ledStripe.setBrightness(50);
	ledStripe.setPixelColor(0, ledStripe.Color(0, 0, 0));
	ledStripe.show();
    	
}


void Test::testOutputWithLedStripe(int pos, int r, int g, int b){

    ledStripe.setPixelColor(0, ledStripe.Color(r, g, b));
    ledStripe.show();
	
}

