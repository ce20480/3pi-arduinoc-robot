// this #ifndef stops this file from being included more than once by the compiler
#ifndef _STATECHECKER_H
# define _STATECHECKER_H

# include "motors.h"
# include "linesensor.h"

Motors_c Motors;
LineSensor_c Linesensor;

class StateChecker_c {

  public:

    // put variables here if wanted and they will be public out of class

    typedef enum {

      OFF_LINE = 0,
      ON_LINE = 1,
      RIGHT_TURN = 2,

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


    int UpdateState( int state ) {

      unsigned long elapsed_time;

      unsigned long right_elapsed_time;

      right_elapsed_time = Linesensor.ReadRightmostSensor();

      elapsed_time = Linesensor.FindElapsedTime();

      if ( right_elapsed_time >= 1300 ) {

        state = RIGHT_TURN;

      } else if ( elapsed_time >= 1200 ) {

        state = ON_LINE;

      } else {

        state = OFF_LINE;

      }

      return state;

    }

    void UpdateAction( int state ) {

      if (state == ON_LINE) {
        
        Linesensor.DigitalLineFollowingLeft();
        
      }

      if (state == OFF_LINE) {
        
        Motors.TurnLeft( 15, 15 );
        
      }

      if (state == RIGHT_TURN) {   
             
        Motors.TurnRight( 150, 150 );
        
      }
    }

};










#endif
