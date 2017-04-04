#include "Transition.h"


/******************************************************************************/
/* PUBLIC METHODS                                                             */
/******************************************************************************/

Transition::Transition(){
	logger.debug("TRANS::Trans\n");
}

void Transition::reset(){

	logger.debug("TRANS::reset\n");

	transitionToInactive();
}

int Transition::getTransitionToPerform(State state){

	logger.debug("TRANS::getTrans2Perform\n");

	return inputOutputUtils.getTransitionToPerform(state);
}

void Transition::transitionToInactive(){

	logger.debug("TRANS::trans2Inactive\n");

	openMitten();
	openForefinger();
	openThumb();
}

void Transition::transitionToIdle(){

	logger.debug("TRANS::trans2Idle\n");

	openMitten();
	openForefinger();
	openThumb();
}

void Transition::transitionToTongs(){

	logger.debug("TRANS::trans2Tongs\n");

	openMitten();
	closeForefinger();
	closeThumb();
}

void Transition::transitionToFinger(){

	logger.debug("TRANS::trans2Finger\n");

	closeMitten();
	openForefinger();
	closeThumb();
}

void Transition::transitionToClose(){

	logger.debug("TRANS::trans2Close\n");

	closeMitten();
	closeForefinger();
	openThumb();
}

void Transition::transitionToFist(){

	logger.debug("TRANS::trans2Fist\n");

	closeMitten();
	closeForefinger();
	closeThumb();
}

/******************************************************************************/
/* PRIVATE METHODS                                                            */
/******************************************************************************/

  void Transition::openMitten(){

    logger.debug("TRANS::openMitten\n");
    inputOutputUtils.openMitten();
  }

  void Transition::closeMitten(){

    logger.debug("TRANS::closeMitten\n");
    inputOutputUtils.closeMitten();
  }

  void Transition::openForefinger(){

    logger.debug("TRANS::openForefinger\n");
    inputOutputUtils.openForefinger();
  }

  void Transition::closeForefinger(){

    logger.debug("TRANS::closeForefinger\n");
    inputOutputUtils.closeForefinger();
  }

  void Transition::openThumb(){

    logger.debug("TRANS::openThumb\n");
    inputOutputUtils.openThumb();
  }

  void Transition::closeThumb(){

    logger.debug("TRANS::closeThumb\n");
    inputOutputUtils.closeThumb();
  }
