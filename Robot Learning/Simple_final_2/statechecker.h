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
      RIGHT_TURN = 3,
      GO_HOME = 4,

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

    //    void do_PID(unsigned long elapsed_time_2) {
    //      long diff_e_left;
    //      float left_speed;
    //
    //      long diff_e_right;
    //      float right_speed;
    //
    //      diff_e_left = count_e_left - count_left_last;
    //      count_left_last = count_e_left;
    //
    //      diff_e_right = count_e_right - count_right_last;
    //      count_right_last  = count_e_right;
    //
    //      left_speed = (float)diff_e_left;
    //      left_speed /= (float)elapsed_time_2;
    //
    //      right_speed = (float)diff_e_right;
    //      right_speed /= (float)elapsed_time_2;
    //
    //      avg_right_speed = ( avg_right_speed * 0.7 ) + ( right_speed * 0.3 );
    //
    //      avg_left_speed = ( avg_left_speed * 0.7 ) + ( left_speed * 0.3 );
    //
    //      float pwm_right;
    //
    //      float pwm_left;
    //
    //      float demand_right = 0.23;
    //      float demand_left = 0.23;
    //
    //      // ( measurement, demand_right )
    //      pwm_left = spd_pid_left_class.update( left_speed, demand_left );
    //      pwm_right = spd_pid_right_class.update( right_speed, demand_right );
    //
    //      Motors_class.setMotorPower( pwm_left, pwm_right );
    //    }


    int UpdateState( volatile int state, volatile float X_pos, volatile float Y_pos, unsigned long elapsed_time ) {

      //      timer += (float)elapsed_time;

      //      if ( timer < 6000 ) {

      unsigned long elapsed_time_2;

      unsigned long right_elapsed_time;

      right_elapsed_time = Linesensor.ReadRightmostSensor();

      unsigned long middle_elapsed_time = Linesensor.ReadMiddleSensor();

      elapsed_time_2 = Linesensor.FindElapsedTime();

      if ( state == GO_HOME ) {

        state = GO_HOME;

      } else {

        if ( right_elapsed_time >= 1100 ) {

          state = RIGHT_TURN;
          digitalWrite(LED_BUILTIN, LOW);


          //        } else if ( middle_elapsed_time >= 1200 ) {
        } else if ( elapsed_time_2 >= 1200 ) {

          state = ON_LINE;
          digitalWrite(LED_BUILTIN, HIGH);

        } else {

          state = OFF_LINE;
          digitalWrite(LED_BUILTIN, LOW);


        }
      }
      return state;
    }

    void UpdateAction( int state, unsigned long elapsed_time ) {

      if (state == ON_LINE) {

        Linesensor.DigitalLineFollowingLeft();

      }

      if (state == OFF_LINE) {

        Motors.TurnLeft( 20, 20 );

      }

      if (state == RIGHT_TURN) {

        Motors.TurnRight( 150, 150 );

      }

      if (state == BEGIN) {

      }

      if (state == GO_HOME) {

      }
    }

};










#endif
