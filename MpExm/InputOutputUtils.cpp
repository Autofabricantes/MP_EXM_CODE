#include "InputOutputUtils.h"

/******************************************************************************/
/* INITIALIZATION INPUT METHODS                                               */
/******************************************************************************/

void InputOutputUtils::initializeInputElements() {

	logger.debug("IOUTILS::initInput");

	//logger.debug("IOUTILS::initInput (50segs aprox.)\n");

	//myowareSensorController1 = MyoControl(CONTROL_INPUT_MYOWARE_SENSOR_1);
	//myowareSensorController2 = MyoControl(CONTROL_INPUT_MYOWARE_SENSOR_2);
  
	//logger.info("IOUTILS::initializeInputElements - Calibrate myoware Sensor 1\n");
	//myowareSensorController1.calibration();
	//logger.info("IOUTILS::initializeInputElements - Calibrate myoware Sensor 2\n");
	//myowareSensorController2.calibration();
	
}

void InputOutputUtils::resetInputElements() {

	logger.debug("IOUTILS::resetInput\n");

	//myowareSensorController1.calibration();
	//myowareSensorController2.calibration();

 

}

/******************************************************************************/
/* INITIALIZATION OUTPUT METHODS                                              */
/******************************************************************************/

void InputOutputUtils::initializeOutputElements() {


	logger.info("IOUTILS::initOutput\n");

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
  
	logger.info("IOUTILS::initOutput-Init mitten\n");
	initialFingerControl(MITTEN, CONTROL_INPUT_POTENTIOMETER_MITTEN);
	delay(1000);

	logger.info("IOUTILS::initOutput-Init forefinger\n");
	initialFingerControl(FOREFINGER, CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	delay(1000);

	logger.info("IOUTILS::initOutput-Init thumb\n");
	initialFingerControl(THUMB, CONTROL_INPUT_POTENTIOMETER_THUMB);
	delay(1000);

}

void InputOutputUtils::resetOutputElements() {

	logger.debug("IOUTILS::resetOutput\n");

	initializeOutputElements();

}


/******************************************************************************/
/* FINGERS POSITION                                                           */
/******************************************************************************/

int InputOutputUtils::getMittenPosition() {

	// TODO: What happens if finger position is diferent to current position?
	// Tenedremos que revisar en que posicion se encuentar el dedo realmente para
	// restaurar la posicion si es necesario.
	int mittenPosition = currentState.getMittenPosition();

	logger.info("IOUTILS::getMittenPos: %i\n", mittenPosition);

	return mittenPosition;

}

int InputOutputUtils::getForefingerPosition() {

	// TODO: What happens if finger position is diferent to current position?
	// Tenedremos que revisar en que posicion se encuentar el dedo realmente para
	// restaurar la posicion si es necesario.
	int forefingerPosition = currentState.getForefingerPosition();

	logger.debug("IOUTILS::getForefingerPos: %i\n", forefingerPosition);

	return forefingerPosition;
}

int InputOutputUtils::getThumbPosition() {

	// TODO: What happens if finger position is diferent to current position?
	// Tenedremos que revisar en que posicion se encuentar el dedo realmente para
	// restaurar la posicion si es necesario.
	int thumbPosition = currentState.getThumbPosition();

	logger.debug("getThumbPos: %i\n", thumbPosition);

	return thumbPosition;

}

/******************************************************************************/
/* TRANSITIONS                                                                */
/******************************************************************************/

int InputOutputUtils::getTransitionToPerform(State state) {

	logger.debug("IOUTILS::getTrans\n");

	currentState = state;

	/*
	// Prueba para los senores Myoware estado ACTIVO/INACTIVO
	boolean activation1 = myowareSensorController1.activation();
	logger.info("IOUTILS::myowareSensorController1 - activation: %d\n", activation1);
	boolean activation2 = myowareSensorController2.activation();
	logger.info("IOUTILS::myowareSensorController2 - activation: %d\n", activation2);

	int transitionTo = 0;
	if(activation1 || activation2){
		transitionTo = STATE_FIST;
	}else{
		transitionTo = STATE_IDLE;
	}


	// Pruba sensores Myoware todos los estados
	int transitionTo = 0;
	if (!activation1 && !activation2){
		logger.info("IOUTILS::IDLE\n");
		transitionTo = STATE_IDLE;
	}else if (!activation1 && activation2) {
		logger.info("IOUTILS::FINGERS\n");
		transitionTo = STATE_FINGER;
	} else if (activation1 && !activation2) {
		logger.info("IOUTILS::TONGS\n");
		transitionTo = STATE_TONGS;
	} else {
		logger.info("IOUTILS::FIST\n");
		transitionTo = STATE_FIST;
	}
	*/
	
	// TODELETE: Funcionalidad que permite hacer tests con el motor
	// sin depender de los sensores

	int transitionTo = 0;

	// Secuencial
	//static int i = 0;
	//transitionTo = ((i++)%STATES_NUMBER);

	// Menu
	transitionTo = test.testInputForTransition();

	return transitionTo;
	
}


void InputOutputUtils::openMitten() {

	logger.debug("IOUTILS::openMitten\n");

    if(getMittenPosition() == CLOSE){
		  logger.info("IOUTILS::openMitten-OPEN\n");
		  fingerControl(MITTEN, OPEN, CONTROL_INPUT_POTENTIOMETER_MITTEN);
	  }

}

void InputOutputUtils::closeMitten() {

	logger.debug("IOUTILS::closeMitten\n");

	if(getMittenPosition() == OPEN){
		logger.info("IOUTILS::closeMitten-CLOSE\n");
		fingerControl(MITTEN, CLOSE, CONTROL_INPUT_POTENTIOMETER_MITTEN);
	}
}

void InputOutputUtils::openForefinger() {

	logger.debug("IOUTILS::openForefinger\n");

	if(getForefingerPosition() == CLOSE){
		logger.debug("IOUTILS::openForefinger-OPEN\n");
		fingerControl(FOREFINGER, OPEN, CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	}
}

void InputOutputUtils::closeForefinger() {

	logger.debug("IOUTILS::closeForefinger\n");

	if(getForefingerPosition() == OPEN){
		logger.debug("IOUTILS::closeForefinger-CLOSE\n");
		fingerControl(FOREFINGER, CLOSE,CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	}
}

void InputOutputUtils::openThumb() {

	logger.debug("IOUTILS::openThumb\n");

	if(getThumbPosition() == CLOSE){
		logger.debug("IOUTILS::openThumb-OPEN\n");
		fingerControl(THUMB, OPEN, CONTROL_INPUT_POTENTIOMETER_THUMB);
	}

}

void InputOutputUtils::closeThumb() {

	logger.debug("IOUTILS::closeThumb\n");

	if(getThumbPosition() == CLOSE){
		logger.debug("IOUTILS::closeThumb-CLOSE\n");
		fingerControl(THUMB,CLOSE,CONTROL_INPUT_POTENTIOMETER_THUMB);
	}

}


/******************************************************************************/
/* PCB CONTROLS                                                               */
/******************************************************************************/


void InputOutputUtils::initialFingerControl(int motorId,  int controlId){

	logger.info("IOUTILS::fingerControl\n");

	int initialPosition = multiplexorRead(controlId);
	int finalPosition = initialPosition;
	logger.info("IOUTILS::fingerControl-Initial pos: %d\n", initialPosition);

	if(finalPosition < 200){
		while(finalPosition < 200){
			motorControl(motorId, OPEN, 100);
			delay(100);
			finalPosition = multiplexorRead(controlId);
		}
		motorControl(motorId, OPEN, MOTOR_SPEED_MIN);
   
	}else if(finalPosition > 500){
		while(finalPosition > 200){
			motorControl(motorId, CLOSE, 100);
			delay(100);
			finalPosition = multiplexorRead(controlId);
		}
		motorControl(motorId, CLOSE, MOTOR_SPEED_MIN);
	}

	logger.info("IOUTILS::fingerControl-finalPos: %d\n", finalPosition);

}


void InputOutputUtils::fingerControl(int motorId, int motorDir, int controlId){

	logger.info("IOUTILS::fingerControl\n");

	int initialPosition = multiplexorRead(controlId);
	int finalPosition = initialPosition;
	logger.info("IOUTILS::fingerControl-Initial pos: %d\n", initialPosition);

	if((finalPosition > 200) && (motorDir == OPEN)){

		motorControl(motorId, OPEN , MOTOR_SPEED);
		delay(1500);
		motorControl(motorId, OPEN, MOTOR_SPEED_MIN);
		finalPosition = multiplexorRead(controlId);

	}else if ((finalPosition < 800) && (motorDir == CLOSE)){

		motorControl(motorId, CLOSE , MOTOR_SPEED);
		delay(1500);
		motorControl(motorId, CLOSE, MOTOR_SPEED_MIN);
		finalPosition = multiplexorRead(controlId);

	}else{
		initialFingerControl(motorId, controlId);
    fingerControl(motorId, motorDir, controlId);
	}
  
	logger.info("IOUTILS::fingerControl-Final pos: %d\n", finalPosition);
      
}



void InputOutputUtils::motorControl(int motorID, int motorDir, int motorSpeed) {

   logger.info("IOUTILS::motorControl\n");

	// Forward Direction --> CLOSE --> 1
    // 1024 --> 0 (decrements)
	if (motorDir) { 
		logger.info("IOUTILS::motorControl-forward direction-CLOSE\n");
		digitalWrite(MOTOR_CONTROL_MATRIX[motorID][1], LOW);
		analogWrite(MOTOR_CONTROL_MATRIX[motorID][0], motorSpeed);
	// Backward Direction --> OPEN --> 0
	// 0 --> 1024 (increments)
	} else {
		logger.info("IOUTILS::motorControl-backward direction-OPEN\n");
		digitalWrite(MOTOR_CONTROL_MATRIX[motorID][1], HIGH);
		analogWrite(MOTOR_CONTROL_MATRIX[motorID][0], (MOTOR_SPEED_MAX - motorSpeed));
	}
  
}


int InputOutputUtils::multiplexorRead(int controlId){

	// Main Multiplexer (vs Acc Multiplexer)
		
	// Lecture Sensors through 74HC4051 Multiplexer
	// Entry channel selection for 74HC4051
	
	logger.info("IOUTILS::multiplexorRead-input[%i]\n", controlId);

	int cA = controlId & 0x01;   
	int cB = (controlId>>1) & 0x01;     
	int cC = (controlId>>2) & 0x01;   
	
	digitalWrite(MUX_A, cA);
	digitalWrite(MUX_B, cB);
	digitalWrite(MUX_C, cC);
  
	int readedValue = analogRead(MUX_MAIN);

	logger.info("IOUTILS::multiplexorRead-output[%i]\n", readedValue);
	
	return readedValue;

}

InputOutputUtils inputOutputUtils = InputOutputUtils();
