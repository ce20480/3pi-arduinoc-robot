// this #ifndef stops this file from being included more than once by the compiler
#ifndef _STATECHECKER_H
# define _STATECHECKER_H

# include "motors.h"
# include "linesensor.h"

Motors_c Motors;
LineSensor_c Linesensor;
Kinematics_c Kinematics;

float timer = 0;

class StateChecker_c {

  public:

    // put variables here if wanted and they will be public out of class

    typedef enum {

      BEGIN = 0,
      OFF_LINE = 1,
      ON_LINE = 2,
      RIGHT_TURN_STRONG = 3,
      GO_HOME = 4,

      JOIN_LINE = 5,
      RIGHT_TURN_WEAK = 6

    } STATE;

    // Constructor must exist
    StateChecker_c() {
    }

    void initialise() {

      // Start Serial, wait to connect, print a debug message.
      Serial.begin(9600);
      delay(1500);
      Serial.println("***RESET***");

    }

    int UpdateState( volatile int state, volatile float X_pos, volatile float Y_pos, unsigned long elapsed_time ) {

      unsigned long elapsed_time_2;

      unsigned long right_elapsed_time;

      right_elapsed_time = Linesensor.ReadRightmostSensor();

      unsigned long left_elapsed_time = Linesensor.ReadLeftmostSensor();

      unsigned long middle_elapsed_time = Linesensor.ReadMiddleSensor();

      elapsed_time_2 = Linesensor.FindElapsedTime();

      if ( middle_elapsed_time >= 1200 )  {

        if ( right_elapsed_time >= 1600 and left_elapsed_time >= 1800 ) {

          state = RIGHT_TURN_STRONG;

        } else if ( right_elapsed_time >= 1600 ) {

          state = RIGHT_TURN_STRONG;

        } else {
          
          state = ON_LINE;
          
        }

      } else if ( right_elapsed_time >= 1400 ) {
        //      } else if ( elapsed_time_2 >= 1200 ) {

        state = RIGHT_TURN_WEAK;

      } else {

        if ( state != JOIN_LINE ) {

          state = OFF_LINE;

        }

      }

      return state;

    }



};










#endif
