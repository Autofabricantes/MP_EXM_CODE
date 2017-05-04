#include <Arduino.h>

#include "Logging.h"

#ifndef CONSTANTS_H
#define CONSTANTS_H

/*****************************************************************************/
/* GENERAL                                                                   */
/*****************************************************************************/

// Identifies open position
#define OPEN                  0
// Identifies close position
#define CLOSE                 1

// Number of finger elements
#define FINGERS               3
// Identifies mitten fingers
#define MITTEN                0
// Identifies forefinger finger 
#define FOREFINGER            1
// Identifies thumb finger
#define THUMB                 2

/*************************************************************************/
/* STATES                                                                */
/*************************************************************************/
// State Number
#define STATES_NUMBER                  6
// Inactive state (inactivo)
#define STATE_INACTIVE                 0
// Resting/Initial state. Open hand
#define STATE_IDLE                     1
// Tongs state. Thumb + forefinger
#define STATE_TONGS                    2
// Finger state. Forefinger
#define STATE_FINGER                   3
// Close state. Mitten + forefinger
#define STATE_CLOSE                    4
// Fist state. Mitten + forefinger + thumb
#define STATE_FIST                     5


/*************************************************************************/
/* TRANSITIONS                                                           */
/*************************************************************************/
// Transition Number
//#define TRANSITIONS_NUMBER             6

// Transition to inactive state
#define INVALID_TRANSITION            -1
// Transition to inactive state
#define TRANSITION_TO_INACTIVE         0
// Transition to resting/initial state
#define TRANSITION_TO_IDLE             1
// Transition to Tongs state
#define TRANSITION_TO_TONGS            2
// Transition to finger state
#define TRANSITION_TO_FINGER           3
// Transition to close state
#define TRANSITION_TO_CLOSE            4
// Transition to fist state
#define TRANSITION_TO_FIST             5

// Fingers position matrix from states definition
// TODO: Considerar todos los posibles estados o eliminar estados 
//       segun necesidades

const int FINGER_POSITION_MATRIX[STATES_NUMBER][FINGERS] = {

    // MITTEN   FORE  THUMB
    { OPEN,  OPEN,  OPEN  }, // STATE_INACTIVE
    { OPEN,  OPEN,  OPEN  }, // STATE_IDLE
    { OPEN,  CLOSE, CLOSE }, // STATE_TONGS
    { CLOSE, OPEN,  CLOSE }, // STATE_FINGER
    { CLOSE, CLOSE, OPEN  }, // STATE_CLOSE
    { CLOSE, CLOSE, CLOSE }  // STATE_FIST
};

/*************************************************************************/
/* MOTORS                                                                */
/*************************************************************************/

// Min Value for motors speed
#define MOTOR_SPEED_MIN   0
// Max Value for motors speed
#define MOTOR_SPEED_MAX  255
// CONFIGURABLE VALUE: Motor Speed
#define MOTOR_SPEED      100


/*****************************************************************************/
/* PIN DESCRIPTION                                                           */
/*****************************************************************************/

// OUTPUT pin for LED RGB
#define PIN_OUTPUT_LED_RGB                          13
#define PIN_OUTPUT_LEDSTRIPE_RGB                    10

// INPUT pin for tactile switch
//#define PIN_INPUT_SWITH                            1
// OUTPUT pin for bluetoothpe
//#define PIN_OUTPUT_BLUETOOTH                      15                            

// Accesory Board Detection
//#define ACC_BRD
// Multiplexer Control Crazy Pionut Assignement A
#define MUX_A                                       16 
// Multiplexer Control Crazy Pionut Assignement B
#define MUX_B                                       14 
// Multiplexer Control Crazy Pionut Assignement C
#define MUX_C                                       15 
// Main Board Multiplexer Output
#define MUX_MAIN                                     0
// Acc Board Multiplexer Output 
#define MUX_ACC                                      1 

// INPUT pin for mitten related potentiometer
#define PIN_OUTPUT_POTENTIOMETER_MITTEN              2
// INPUT pin for forefinger related potentiometer
#define PIN_OUTPUT_POTENTIOMETER_FOREFINGER          4
// INPUT pin for thumb related potentiometer
#define PIN_OUTPUT_POTENTIOMETER_THUMB               6

// INPUT pin for mitten related amperimeter
#define PIN_OUTPUT_CURRENT_SENSOR_MITTEN             3
// INPUT pin for forefinger related amperimeter
#define PIN_OUTPUT_CURRENT_SENSOR_FOREFINGER         5
// INPUT pin for thumb related amperimeter
#define PIN_OUTPUT_CURRENT_SENSOR_THUMB              7

// OUTPUT pin for mitten related motor
#define PIN_OUTPUT_MOTOR_MITTEN_PWM                  9
#define PIN_OUTPUT_MOTOR_MITTEN                      8
// OUTPUT pin for forefinger related motor
#define PIN_OUTPUT_MOTOR_FOREFINGER_PWM              5
#define PIN_OUTPUT_MOTOR_FOREFINGER                  7
// OUTPUT pin for thumb related motor
#define PIN_OUTPUT_MOTOR_THUMB_PWM                   3
#define PIN_OUTPUT_MOTOR_THUMB                       4

//Motor Control Matrix
const int MOTOR_CONTROL_MATRIX[FINGERS][2] =
   {{PIN_OUTPUT_MOTOR_MITTEN_PWM, PIN_OUTPUT_MOTOR_MITTEN},
    {PIN_OUTPUT_MOTOR_FOREFINGER_PWM, PIN_OUTPUT_MOTOR_FOREFINGER},
    {PIN_OUTPUT_MOTOR_THUMB_PWM, PIN_OUTPUT_MOTOR_THUMB}};   


// Multiplexor Control Matrix
// MYO_0
#define CONTROL_INPUT_MYOWARE_SENSOR_1          0
//MYO_1
#define CONTROL_INPUT_MYOWARE_SENSOR_2          1
// MPOT_0
#define CONTROL_INPUT_POTENTIOMETER_MITTEN      2
// CS_0
#define CONTROL_INPUT_CURRENT_SENSOR_MITTEN     3
// MPOT_1
#define CONTROL_INPUT_POTENTIOMETER_FOREFINGER  4
// CS_1
#define CONTROL_INPUT_CURRENT_SENSOR_FOREFINGER 5
// MPOT_2
#define CONTROL_INPUT_POTENTIOMETER_THUMB       6
// CS_2
#define CONTROL_INPUT_CURRENT_SENSOR_THUMB      7


// PID FUNCTION

// Tuning PID parameters
//Initial Proportional Gain
#define PID_KP 1
//Initial Integral Gain
#define PID_KI 1
//Initial Differential Gain
#define PID_KD 1

// Variable that determines how fast our PID loop runs
#define PID_LOOP_TIME 300


#endif
