#include "StateMachine.h"
#include "Constants.h"


/******************************************************************************/
/* PUBLIC METHODS                                                             */
/******************************************************************************/

void StateMachine::start(){

  logger.debug("STM::start\n");

  state = State();

  state.setCurrentState(TRANSITION_TO_INACTIVE);
  transition = Transition();

  test.testInitializeLedStripe();
  
}

void StateMachine::executeTransition(){

    logger.debug("STM::executeTrans\n");
    
    int currentState = state.getCurrentState();

    int transitionToPeform = transition.getTransitionToPerform(state);

    logger.debug("STM::executeTrans: %i\n", transitionToPeform);

    switch(transitionToPeform){

      case INVALID_TRANSITION:
      break;

      case TRANSITION_TO_INACTIVE:
        transition.transitionToInactive();
        test.testOutputWithLedStripe(0,0,0,0);
        state.setCurrentState(STATE_INACTIVE);
      break;

      case TRANSITION_TO_IDLE:
        transition.transitionToIdle();
        test.testOutputWithLedStripe(0,102,204,0);
        state.setCurrentState(STATE_IDLE);
      break;

      case TRANSITION_TO_TONGS:
        if(state.getCurrentState() != STATE_INACTIVE){
          transition.transitionToTongs();
          test.testOutputWithLedStripe(0,0,128,255);
          state.setCurrentState(STATE_TONGS);
        }
      break;

      case TRANSITION_TO_FINGER:
        if(state.getCurrentState() != STATE_INACTIVE){
          transition.transitionToFinger();
          test.testOutputWithLedStripe(0,255,255,0);
          state.setCurrentState(STATE_FINGER);
        }
      break;

      case TRANSITION_TO_CLOSE:
        if(state.getCurrentState() != STATE_INACTIVE){
          transition.transitionToClose();
          test.testOutputWithLedStripe(0,153,0,153);
          state.setCurrentState(STATE_CLOSE);
        }
      break;

      case TRANSITION_TO_FIST:
        if(state.getCurrentState() != STATE_INACTIVE){
          transition.transitionToFist();
          test.testOutputWithLedStripe(0,204,0,0);
          state.setCurrentState(STATE_FIST);
        }
      break;

      default:
      break;

    }
}

void StateMachine::reset(){

  logger.debug("STM::reset\n");

  state.reset();
  transition.reset();

}
