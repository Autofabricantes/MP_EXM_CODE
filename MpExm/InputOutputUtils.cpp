#include "InputOutputUtils.h"

/******************************************************************************/
/* INITIALIZATION INPUT METHODS                                               */
/******************************************************************************/

void InputOutputUtils::initializeInputElements() {

	logger.debug("IOUTILS::initInput"); 	
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
  
	// TOINCLUDEAGAIN
	//logger.info("IOUTILS::initOutput-Initialize mitten\n");
	//initialFingerControl(MITTEN, CONTROL_INPUT_POTENTIOMETER_MITTEN);
	//delay(1000);

	//logger.info("IOUTILS::initOutput-Init forefinger\n");
	//initialFingerControl(FOREFINGER, CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	//delay(1000);

	//logger.info("IOUTILS::initOutput-Init thumb\n");
	//initialFingerControl(THUMB, CONTROL_INPUT_POTENTIOMETER_THUMB);
	//delay(1000);

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

	logger.info("IOUTILS::getMittenPosition: %i\n", mittenPosition);

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
		logger.info("IOUTILS::openForefinger-OPEN\n");
		//fingerControl(FOREFINGER, OPEN, CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	}
}

void InputOutputUtils::closeForefinger() {

	logger.debug("IOUTILS::closeForefinger\n");

	if(getForefingerPosition() == OPEN){
		logger.info("IOUTILS::closeForefinger-CLOSE\n");
		//fingerControl(FOREFINGER, CLOSE,CONTROL_INPUT_POTENTIOMETER_FOREFINGER);
	}
}

void InputOutputUtils::openThumb() {

	logger.info("IOUTILS::openThumb\n");

	if(getThumbPosition() == CLOSE){
		logger.info("IOUTILS::openThumb-OPEN\n");
		//fingerControl(THUMB, OPEN, CONTROL_INPUT_POTENTIOMETER_THUMB);
	}

}

void InputOutputUtils::closeThumb() {

	logger.info("IOUTILS::closeThumb\n");

	if(getThumbPosition() == CLOSE){
		logger.info("IOUTILS::closeThumb-CLOSE\n");
		//fingerControl(THUMB,CLOSE,CONTROL_INPUT_POTENTIOMETER_THUMB);
	}

}


/******************************************************************************/
/* PCB CONTROLS                                                               */
/******************************************************************************/

void InputOutputUtils::initialFingerControl(int motorId,  int controlId){

	//initialFingerControlTime(motorId, controlId);
	initialFingerControlPID(motorId, controlId);
}

void InputOutputUtils::initialFingerControlTime(int motorId,  int controlId){

	logger.info("IOUTILS::initialFingerControlTime\n");

	int initialPosition = multiplexorRead(controlId);
	int finalPosition = initialPosition;
	logger.info("IOUTILS::initialFingerControlTime-initialPos: %d\n", initialPosition);

	if(finalPosition < 200){
		while(finalPosition < 200){
			motorControl(motorId, CLOSE, 100);
			delay(100);
			finalPosition = multiplexorRead(controlId);
		}
	}

	if(finalPosition > 800){
		while(finalPosition > 800){
			motorControl(motorId, OPEN, 100);
			delay(100);
			finalPosition = multiplexorRead(controlId);
		}
	}

	logger.info("IOUTILS::initialFingerControlTime-finalPos: %d\n", finalPosition);

}

// TODO - No es necesaria
void InputOutputUtils::initialFingerControlPID(int motorId,  int controlId){

	logger.info("IOUTILS::initialFingerControlPID\n");

	double setpoint, input, output;
	PID pid;
	int motorDir;

	input = map(multiplexorRead(controlId), 0, 1024, MOTOR_SPEED_MIN, MOTOR_SPEED);
	//input = multiplexorRead(controlId);
	logger.info("IOUTILS::initialFingerControlPID - input: %f\n", input);

	setpoint = 0;
	logger.info("IOUTILS::initialFingerControlPID - initialization setpoint: %f\n", setpoint);

	// Initialize PID
	/*
	if(input < 200){
		pid = PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
		motorDir = OPEN;
	}else{
		pid = PID(&input, &output, &setpoint, kp, ki, kd, REVERSE);
		motorDir = CLOSE;
	}*/

	if(input > 0){
		pid = PID(&input, &output, &setpoint, PID_KP, PID_KI, PID_KD, REVERSE);
		motorDir = OPEN;
	}

	//Sets the sample rate
	pid.SetSampleTime(PID_LOOP_TIME);
	//Turn on the PID loop
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(0,MOTOR_SPEED);

	while(input > 0){

		input = map(multiplexorRead(controlId), 0, 1024, MOTOR_SPEED_MIN, MOTOR_SPEED);
		//input = multiplexorRead(controlId);

		delay(300);
		pid.Compute();

		motorControl(motorId, motorDir, round(output));

		input = map(multiplexorRead(controlId), 0, 1024, MOTOR_SPEED_MIN, MOTOR_SPEED);
		//input = multiplexorRead(controlId);
		logger.info("IOUTILS::initialFingerControlPID - loop input: %f\n", input);
		logger.info("IOUTILS::initialFingerControlPID - loop output: %f\n", output);

	}

  logger.info("IOUTILS::initialFingerControlPID - Stop motor \n");
	motorControl(motorId, motorDir, MOTOR_SPEED_MIN);

	//Turn off the PID loop
	pid.SetMode(MANUAL);

}


void InputOutputUtils::fingerControl(int motorId, int motorDir, int controlId){

	logger.info("IOUTILS::fingerControl\n");

	//fingerControlTime(motorId, motorDir, controlId);
	fingerControlPID(motorId, motorDir, controlId);

}

void InputOutputUtils::fingerControlTime(int motorId, int motorDir, int controlId){

	logger.info("IOUTILS::fingerControlTime\n");

	int initialPosition = multiplexorRead(controlId);
	int finalPosition = initialPosition;
	logger.info("IOUTILS::fingerControlTime-Initial pos: %d\n", initialPosition);

	if((finalPosition < 200) || (finalPosition > 800)){

		initialFingerControl(motorId, controlId);
		logger.info("IOUTILS::fingerControlTime-Execute again finger control");
		fingerControl(motorId, motorDir, controlId);

	}else if((finalPosition) > 200 && (motorDir == OPEN)){

		motorControl(motorId, OPEN , MOTOR_SPEED);
		delay(100);
		motorControl(motorId, OPEN, MOTOR_SPEED_MIN);
		finalPosition = multiplexorRead(controlId);

	}else if ((finalPosition < 800) && (motorDir == CLOSE)){
		motorControl(motorId, CLOSE , MOTOR_SPEED);
		delay(100);
		motorControl(motorId, CLOSE, MOTOR_SPEED_MIN);
		finalPosition = multiplexorRead(controlId);

	}

	logger.info("IOUTILS::fingerControlTime-Final pos: %d\n", finalPosition);
      
}

void InputOutputUtils::fingerControlPID(int motorId, int motorDir, int controlId){

    logger.info("IOUTILS::fingerControlPID\n");

    double setpoint, input, output;
    PID pid;

    input = map(multiplexorRead(controlId), 0, 1023, 0, MOTOR_SPEED);
	//input = multiplexorRead(controlId);

    logger.info("IOUTILS::fingerControlPID - input: %f\n", input);

    if (motorDir == OPEN){

    	setpoint = MOTOR_SPEED_MIN;
    	logger.info("IOUTILS::fingerControlPID - OPEN - final setpoint: %f\n", setpoint);
  
    	// Initialize PID
    	PID pid = PID(&input, &output, &setpoint, PID_KP, PID_KI, PID_KD, REVERSE);


    }else if (motorDir == CLOSE){

    	setpoint = MOTOR_SPEED;
    	logger.info("IOUTILS::fingerControlPID - CLOSE  - final setpoint: %f\n", setpoint);
  
    	// Initialize PID
    	PID pid = PID(&input, &output, &setpoint, PID_KP, PID_KI, PID_KD, DIRECT);
    }

	  //Sets the sample rate
    pid.SetSampleTime(PID_LOOP_TIME);
    //Turn on the PID loop
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, MOTOR_SPEED);

    while(abs(input - setpoint) > 10){

    	input = map(multiplexorRead(controlId), 0, 1024, 0, MOTOR_SPEED);
    	//input = multiplexorRead(controlId);

    	logger.info("IOUTILS::fingerControlPID - input: %f\n", input);

    	pid.Compute();

    	delay(100);

    	motorControl(motorId, motorDir, round(output));

    	logger.info("IOUTILS::fingerControlPID - output: %f\n", output);

    }

    logger.info("IOUTILS::fingerControlPID - Stopping motor\n");
    motorControl(motorId, motorDir, MOTOR_SPEED_MIN);

    //Turn off the PID loop
    pid.SetMode(MANUAL);

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
		// TODO - Tengo que modificar esto si quiero cambiar el speed
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
