#include "Constants.h"
#include "InputOutputUtils.h"
#include "StateMachine.h"

#define LOGLEVEL LOG_LEVEL_INFOS

// TODOs
// Lector PID para el potenciometro?



StateMachine stateMachine;
int counter = 0;

void setup(){
  
	logger.init(LOGLEVEL, 115200);
    delay(5000);
  
	logger.info("\n---> Setup\n");

  stateMachine.start();

	inputOutputUtils.initializeInputElements();

	inputOutputUtils.initializeOutputElements();
  
}
  
void loop(){

	logger.info("\n---> Loop (%d)\n", counter);
	counter++;
	stateMachine.executeTransition();
	//delay(5000);

}

void reset(){

	logger.debug("\n---> Reset (%d)\n", counter);

	inputOutputUtils.initializeInputElements();

	inputOutputUtils.initializeOutputElements();

	stateMachine.reset();
 
}
