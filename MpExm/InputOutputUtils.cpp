#include "InputOutputUtils.h"

/******************************************************************************/
/* INITIALIZATION INPUT METHODS                                               */
/******************************************************************************/

void InputOutputUtils::initializeInputElements() {

	logger.debug("IOUtils::initInputElmnts (50segs aprox.)\n");

	myowareSensorController1 = MyoControl(CONTROL_INPUT_MYOWARE_SENSOR_1);
	myowareSensorController2 = MyoControl(CONTROL_INPUT_MYOWARE_SENSOR_2);
  
	logger.info("--Calibrate myoware Sensor 1\n");
	myowareSensorController1.calibration();
	logger.info("--Calibrate myoware Sensor 2\n");
	myowareSensorController2.calibration();
	
	test.testInitializeLedStripe();

}

void InputOutputUtils::resetInputElements() {

	logger.debug("IOUtils::resetInputElmnts\n");

	myowareSensorController1.calibration();
	myowareSensorController2.calibration();

}

/******************************************************************************/
/* INITIALIZATION OUTPUT METHODS                                              */
/******************************************************************************/

void InputOutputUtils::initializeOutputElements() {


	logger.info("IOUtils::initOutputElmnts\n");

	// Initialize motors pinout
	pinMode(MUX_A, OUTPUT);
	pinMode(MUX_B, OUTPUT);
	pinMode(MUX_C, OUTPUT);
  
	// Initialize motors pinout
	pinMode(PIN_OUTPUT_MOTOR_MITTEN_PWM, OUTPUT);
	pinMode(PIN_OUTPUT_MOTOR_MITTEN, OUTPUT);
	pinMode(PIN_OUTPUT_MOTOR_FOREFINGER_PWM, OUTPUT);
	pinMode(PIN_OUTPUT_MOTOR_FOREFINGER, OUTPUT);
	pinMode(PIN_OUTPUT_MOTOR_THUMB_PWM, OUTPUT);
	pinMode(PIN_OUTPUT_MOTOR_THUMB, OUTPUT);
  
	logger.info("--Init mitten\n");
	initialFingerControl(MITTEN, CONTROL_INPUT_POTENTIOMETER_MITTEN);
	delay(1000);

	//logger.info("InputOutputUtils - initializeOutputElements - Open forefinger\n");
	//initialFingerControl(FOREFINGER, CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	//delay(1000);

	//logger.info("InputOutputUtils - initializeOutputElements - Open thumb\n");
	//initialFingerControl(THUMB, CONTROL_INPUT_POTENTIOMETER_THUMB);
	//delay(1000);
	
	//test.testOutputWithLedStripe(0,0,0);
	//delay(2000);
	//test.testOutputWithLedStripe(0,255,0);

}

void InputOutputUtils::resetOutputElements() {

	logger.debug("IOUtils::resetOutputElmnts\n");

	initializeOutputElements();

}


/******************************************************************************/
/* FINGERS POSITION                                                           */
/******************************************************************************/

int InputOutputUtils::getMittenPosition() {

	//TODO: What happens if finger position is diferent to current position?
	// Tenedremos que revisar en qu� posicion se encuentar el dedo realmente para
	// restaurar la posicion si es necesario.
	int mittenPosition = currentState.getMittenPosition();

	logger.info("IOUtils::getMittenPos--Pos: %i\n", mittenPosition);

	return mittenPosition;

}

int InputOutputUtils::getForefingerPosition() {

	// TODO: What happens if finger position is diferent to current position?
	// Tenedremos que revisar en qué posicion se encuentar el dedo realmente para 
	// restaurar la posicion si es necesario.
	int forefingerPosition = currentState.getForefingerPosition();

	logger.debug("IOUtils::getForefingerPos--Pos: %i\n", forefingerPosition);

	return forefingerPosition;
}

int InputOutputUtils::getThumbPosition() {

	//TODO: What happens if finger position is diferent to current position?
	// Tenedremos que revisar en qué posicion se encuentar el dedo realmente para 
	// restaurar la posicion si es necesario.
	int thumbPosition = currentState.getThumbPosition();

	logger.debug("IOUtils::getThumbPos--Pos: %i\n", thumbPosition);

	return thumbPosition;

}

/******************************************************************************/
/* TRANSITIONS                                                                */
/******************************************************************************/

int InputOutputUtils::getTransitionToPerform(State state) {

	logger.debug("IOUtils::getTransToPerform\n");

	currentState = state;

	boolean activation1 = myowareSensorController1.activation();
	logger.info("--myowareSensorContr1 - activation: %d\n", activation1);
	boolean activation2 = myowareSensorController2.activation();
	logger.info("--myowareSensorContr2 - activation: %d\n", activation2);

	int transitionTo = 0;
	if(activation1 || activation2){
		transitionTo = STATE_FIST;
	}else{
		transitionTo = STATE_IDLE;
	}

	/*
	// Misma prueba un poco mas evolucionada
	int transitionTo = 0;
	if (!activation1 && !activation2){
		logger.info("InputOutputUtils - IDLE\n");
		transitionTo = STATE_IDLE;
	}else if (!activation1 && activation2) {
		logger.info("InputOutputUtils - FINGERS\n");
		transitionTo = STATE_FINGER;
	} else if (activation1 && !activation2) {
		logger.info("InputOutputUtils - TONGS\n");
		transitionTo = STATE_TONGS;
	} else {
		logger.info("InputOutputUtils - FIST\n");
		transitionTo = STATE_FIST;
	}
	*/
	
	/*
  	// TODELETE: Funcionalidad que permite hacer tests con el motor
  	// sin depender de los sensores
	static int i = 0;
	int transitionTo = 0;
	transitionTo = ((i++)%STATES_NUMBER);
	*/

	return transitionTo;
	
}


void InputOutputUtils::openMitten() {

	logger.debug("IOUtils::openMitten\n");

    // TODO - Mucho cuidado!!! Estoy hay que recuperarlo
    //if(getMittenPosition() == CLOSE){
		logger.info("--OPEN\n");
		test.testOutputWithLedStripe(0,0,255,0);
		test.testOutputWithLedStripe(1,0,255,0);
		test.testOutputWithLedStripe(2,0,255,0);
		fingerControl(MITTEN, OPEN, CONTROL_INPUT_POTENTIOMETER_MITTEN);
	//}

}

void InputOutputUtils::closeMitten() {

	logger.debug("IOUtils::closeMitten\n");

	// TODO - Mucho cuidado!!! Estoy hay que recuperarlo
	//if(getMittenPosition() == OPEN){
		logger.info("--CLOSE\n");
		test.testOutputWithLedStripe(0,255,0,0);
		test.testOutputWithLedStripe(1,255,0,0);
		test.testOutputWithLedStripe(2,255,0,0);
		fingerControl(MITTEN, CLOSE, CONTROL_INPUT_POTENTIOMETER_MITTEN);
	//}
}

void InputOutputUtils::openForefinger() {

	logger.debug("IOUtils::openForefinger\n");

	if(getForefingerPosition() == CLOSE){
		logger.debug("--OPEN\n");
		test.testOutputWithLedStripe(3,0,255,0);
		//fingerControl(FOREFINGER, OPEN, CONTROL_INPUT_POTENTIOMETER_FOREFINGER);	
	}
}

void InputOutputUtils::closeForefinger() {

	logger.debug("IOUtils::closeForefinger\n");

	if(getForefingerPosition() == OPEN){
		logger.debug("--CLOSE\n");
		test.testOutputWithLedStripe(3,255,0,0);
		//fingerControl(FOREFINGER, CLOSE,CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	}
}

void InputOutputUtils::openThumb() {

	logger.debug("IOUtils::openThumb\n");

	if(getThumbPosition() == CLOSE){
		logger.debug("--OPEN\n");
		test.testOutputWithLedStripe(4,0,255,0);
		//fingerControl(THUMB, OPEN, CONTROL_INPUT_POTENTIOMETER_THUMB);
	}

}

void InputOutputUtils::closeThumb() {

	logger.debug("IOUtils::closeThumb\n");

	if(getThumbPosition() == CLOSE){
		logger.debug("--CLOSE\n");
		test.testOutputWithLedStripe(4,255,0,0);
		//fingerControl(THUMB,CLOSE,CONTROL_INPUT_POTENTIOMETER_THUMB);
	}

}


/******************************************************************************/
/* PCB CONTROLS                                                               */
/******************************************************************************/


void InputOutputUtils::initialFingerControl(int motorId,  int controlId){

	logger.info("IOUtils::fingerControl\n");

	int initialPosition = multiplexorRead(controlId);
	int finalPosition = initialPosition;
	logger.info("--Initial pos: %d\n", initialPosition);

	if(finalPosition < 200){
		while(finalPosition < 200){
			motorControl(motorId, OPEN, 100);
			delay(100);
			finalPosition = multiplexorRead(controlId);
		}
		motorControl(motorId, OPEN, MOTOR_SPEED_MIN);
	}

	if(finalPosition > 800){
		while(finalPosition > 800){
			motorControl(motorId, CLOSE, 100);
			delay(100);
			finalPosition = multiplexorRead(controlId);
		}
		motorControl(motorId, CLOSE, MOTOR_SPEED_MIN);
	}

	logger.info("--finalPos: %d\n", finalPosition);

}


void InputOutputUtils::fingerControl(int motorId, int motorDir, int controlId){

  logger.info("IOUtils::fingerControl\n");

  int initialPosition = multiplexorRead(controlId);
  int finalPosition = initialPosition;
  logger.info("--Initial pos: %d\n", initialPosition);

  if(finalPosition > 200){

	  motorControl(motorId, OPEN , MOTOR_SPEED);
	  delay(500);
	  motorControl(motorId, OPEN, MOTOR_SPEED_MIN);
	  finalPosition = multiplexorRead(controlId);

  }else if (finalPosition < 800){

	  motorControl(motorId, CLOSE , MOTOR_SPEED);
	  delay(500);
	  motorControl(motorId, CLOSE, MOTOR_SPEED_MIN);
	  finalPosition = multiplexorRead(controlId);

  }else{
	  logger.info("--Finger regulation");
	  delay(1000);
	  initialFingerControl(motorId, controlId);
  }
  
  logger.info("--Final pos: %d\n", finalPosition);
      
  /*
	motorControl(motorId, motorDir, MOTOR_SPEED); 
	// Manual Stop
	if(motorDir){
    multiplexorRead(controlId);
    delay(1000);
		//while(multiplexorRead(controlId) > 200);
  }else{
    multiplexorRead(controlId);
    delay(1000);
		//while(multiplexorRead(controlId) < 400);
	}	
	motorControl(motorId, motorDir, MOTOR_SPEED_MIN);
 */

}



void InputOutputUtils::motorControl(int motorID, int motorDir, int motorSpeed) {

   logger.info("IOUtils::motorControl\n");

	// Forward Direction --> CLOSE --> 1
    // 1024 --> 0 (decrements)
	if (motorDir) { 
    logger.info("--forward direction - CLOSE\n");
		digitalWrite(MOTOR_CONTROL_MATRIX[motorID][1], LOW);
		analogWrite(MOTOR_CONTROL_MATRIX[motorID][0], motorSpeed);
	// Backward Direction --> OPEN --> 0
	// 0 --> 1024 (increments)
	} else {
    logger.info("--backward direction - OPEN\n");
		digitalWrite(MOTOR_CONTROL_MATRIX[motorID][1], HIGH);
		analogWrite(MOTOR_CONTROL_MATRIX[motorID][0], (MOTOR_SPEED_MAX - motorSpeed));
	}
  
}


int InputOutputUtils::multiplexorRead(int controlId){

	// Main Multiplexer (vs Acc Multiplexer)
		
	// Lecture Sensors through 74HC4051 Multiplexer
	// Entry channel selection for 74HC4051
	
	logger.info("IOUtils::multiplexorRead - input[%i]\n", controlId);

	int cA = controlId & 0x01;   
	int cB = (controlId>>1) & 0x01;     
	int cC = (controlId>>2) & 0x01;   
	
	digitalWrite(MUX_A, cA);
	digitalWrite(MUX_B, cB);
	digitalWrite(MUX_C, cC);
  
	int readedValue = analogRead(MUX_MAIN);

	logger.info("IOUtils::multiplexorRead - output[%i]\n", readedValue);
	
	return readedValue;

}

InputOutputUtils inputOutputUtils = InputOutputUtils();
