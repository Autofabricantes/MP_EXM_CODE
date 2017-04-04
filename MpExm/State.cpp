#include "State.h"

/******************************************************************************/
/* PUBLIC METHODS                                                             */
/******************************************************************************/

State::State(){

	logger.debug("STATE::State\n");

	currentState = STATE_INACTIVE;
}

int State::getCurrentState(){

	logger.debug("STATE::getCurrentState: %i\n", currentState);

	return currentState;
}

void State::setCurrentState(int state){

	logger.info("STATE::setCurrentState: %i\n", state);

	currentState = state;
}

void State::reset(){

	logger.debug("STATE::reset\n");

	currentState = STATE_INACTIVE;
}


int State::getMittenPosition(){
		
	int fingerPosition   = FINGER_POSITION_MATRIX[currentState][MITTEN];
	logger.info("STATE::getMittenPos-State[%i]-Pos[%i]\n", currentState, fingerPosition);

	return fingerPosition;

}

int State::getForefingerPosition(){
  
	int fingerPosition   = FINGER_POSITION_MATRIX[currentState][FOREFINGER];
	logger.debug("STATE::getForefingerPos-State[%i]-Pos[%i]\n", currentState, fingerPosition);

	return fingerPosition;

}

int State::getThumbPosition(){
  
	int fingerPosition   = FINGER_POSITION_MATRIX[currentState][THUMB];
	logger.debug("STATE::getThumbPos-State[%i]-Pos[%i]\n", currentState, fingerPosition);

	return fingerPosition;

}
