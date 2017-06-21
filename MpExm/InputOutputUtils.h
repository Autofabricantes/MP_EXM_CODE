#ifndef INPUT_OUTPUT_UTILS_H
#define INPUT_OUTPUT_UTILS_H

#include <PID_v1.h>

#include "Constants.h"
#include "MyoControl.h"
#include "State.h"
#include "Test.h"

class InputOutputUtils{

  private:

	int relativePotMittenValue;
	int relativePotForefingerValue;
	int relativePotThumbValue;

	// TODELETE - Test Class
	Test test;

	// INPUT - Myoware Sensor Controllers
	//MyoControl myowareSensorController1;
	//MyoControl myowareSensorController2;

	// State to retrieve current finger's position
    State currentState;
    
    // Motor Control method
    void motorControl(int motorID, int motorDir, int motorSpeed);

    // Multiplexor lecture method
    int multiplexorRead(int controlId);
    int initializePotMultiplexorRead(int controlId);

	// Finger control methods
	void fingerControl(int motorId, int motorDir, int controlId);

	void initialFingerControlTime(int motorId, int controlId);
	void fingerControlTime(int motorId, int motorDir, int controlId);
	void initialFingerControlPID(int motorId, int controlId);
	void fingerControlPID(int motorId, int motorDir, int controlId);

  public:

    /*************************************************************************/
    /* INPUT METHODS                                                         */
    /*************************************************************************/

	void initializeRelativePotsValue();
	int getRelativePotValue(int controlId, int currentValue);

	// TODO - TAKE BACK TO PRIVATE
    void initialFingerControl(int motorId, int controlId);

    // Initialization of INPUT sensors
    void initializeInputElements();

    // Reset of INPUT sensors
    void resetInputElements();

	// Identifies the state selected by user from input elements feedback
    // An interpretation and treatment of readed data from sensors will be  
	// needed to perform the required transition to get the selected state 
  	// without  ambiguity
    // returns: Transition value
    int getTransitionToPerform(State state);
	
    // Detects mitten position from output elements feedback
    // returns: OPEN|CLOSE
	// TODO: Two solutions for fingers position
    //  - Detect where the finger is
    //  - Trust where the state says we are
    int getMittenPosition();

    // Detects forefinger position from output elements feedback
    // returns: OPEN|CLOSE
	// TODO: Two solutions for fingers position
    //  - Detect where the finger is
    //  - Trust where the state says we are
    int getForefingerPosition();

    // Detects thumb position from output elements feedback
    // returns: OPEN|CLOSE
	// TODO: Two solutions for fingers position
    //  - Detect where the finger is
    //  - Trust where the state says we are
    int getThumbPosition();


    /**************************************************************************/
    /* OUTPUT METHODS                                                         */
    /**************************************************************************/

    // Initialization of OUTPUT elements
    void initializeOutputElements();

    // Reset of OUTPUT elements
    void resetOutputElements();

    // Moves mitten to OPEN postion if necesary
    void openMitten();

    // Moves mitten to CLOSE postion if necesary
    void closeMitten();

    // Moves forefinger to OPEN postion if necesary
    void openForefinger();

    // Moves forefinger to CLOSE postion if necesary
    void closeForefinger();

       // Moves thumb to OPEN postion if necesary
    void openThumb();

    // Moves mitten to CLOSE postion if necesary
    void closeThumb();

};

extern InputOutputUtils inputOutputUtils;

#endif
