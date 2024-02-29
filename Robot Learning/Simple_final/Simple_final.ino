#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "statechecker.h"

#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
// of the LED, and toggle it.

unsigned long time_prev; // timestamp
volatile float left_angular_speed;
volatile float right_angular_speed;
float avg_left_speed; // low pass filter of speed
float avg_right_speed;
long count_left_last; // for the difference in encoder counts
long count_right_last; // for the difference in encoder counts
volatile int prev_state;
int begin_state = 0;
const float encoderCountperDegree = 1.0047446274F; // number of encoder counts per degree

typedef enum {

  BEGIN = 0,
  OFF_LINE = 1,
  ON_LINE = 2,
  RIGHT_TURN = 3,
  GO_HOME = 4,
  DONE = 5,

} STATE;

volatile int state = BEGIN;
volatile int prev_state_1;
bool done_before = false;

Motors_c Motors_class;
LineSensor_c Linesensor_class;
StateChecker_c Statechecker_class;
Kinematics_c Kinematics_class;
PID_c spd_pid_left_class;
PID_c spd_pid_right_class;


// put your setup code here, to run once:
void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  // Set LED pin as an output
  pinMode( LED_PIN, OUTPUT );

  time_prev = micros();
  count_left_last = count_e_left;
  count_right_last = count_e_right;

  // Setup encoders when the robot is
  // powered on.
  //  setupEncoder0();
  //  setupEncoder1();
  Linesensor_class.initialise_digital();
  Motors_class.initialise();
  spd_pid_left_class.initialise( 8.0, 0.2, 0.0 ); // 8 2 100
  spd_pid_right_class.initialise( 8.0, 0.2, 0.0 );// 8 2 100
  Kinematics_class.initialise();

  // Set initial state of the LED
  led_state = false;
  spd_pid_left_class.reset();
  spd_pid_right_class.reset();

  volatile int prev_state = state;

  Motors_class.setMotorPower( 100, 100 );


}

void check_counts() {
  Serial.print(state_e_left);
  Serial.print(",");
  Serial.print(count_e_left);
  Serial.print("  ");
  Serial.print(state_e_right);
  Serial.print(",");
  Serial.println(count_e_right);
}

void check_kinematics(volatile float X_pos, volatile float Y_pos, volatile float theta) {
  Serial.print(X_pos);
  Serial.print(", ");
  Serial.print(Y_pos);
  //  Serial.print(", ");
  //  Serial.print(sin(theta));
  //  Serial.print(", ");
  //  Serial.print(cos(theta));
  //  Serial.print(", ");
  //  Serial.print(PI);
  float theta_2 = theta * (180 / PI);
  Serial.print(", ");
  Serial.print(theta_2);
  //  Serial.print(" ,");
  //  Serial.println(theta_r_der);
}

void check_X_pos( float left_speed, float right_speed, float check_distance ) {
  while (Kinematics_class.X_pos < check_distance) {
    //    state = Statechecker_class.UpdateState( state );
    //    Statechecker_class.UpdateAction( state );
    Motors_class.setMotorPower( left_speed, right_speed );
    unsigned long PERIOD = 10;
    Kinematics_class.update(count_e_left, count_e_right);
    check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);
  }
  Motors_class.setMotorPower( 0, 0 );
}

void check_theta( float left_speed, float right_speed, float check_angle ) {
  check_angle = check_angle * (PI / 180);
  while (Kinematics_class.theta < check_angle) {
    //    state = Statechecker_class.UpdateState( state );
    //    Statechecker_class.UpdateAction( state );
    Motors_class.TurnRight( left_speed, right_speed );
    unsigned long PERIOD = 10;
    Kinematics_class.update(count_e_left, count_e_right);
    check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);
  }
  Motors_class.setMotorPower( 0, 0 );
}

void do_PID(unsigned long elapsed_time_2) {
  long diff_e_left;
  float left_speed;

  long diff_e_right;
  float right_speed;

  diff_e_left = count_e_left - count_left_last;
  count_left_last = count_e_left;

  diff_e_right = count_e_right - count_right_last;
  count_right_last  = count_e_right;

  left_speed = (float)diff_e_left;
  left_speed /= (float)elapsed_time_2;

  right_speed = (float)diff_e_right;
  right_speed /= (float)elapsed_time_2;

  avg_right_speed = ( avg_right_speed * 0.7 ) + ( right_speed * 0.3 );

  avg_left_speed = ( avg_left_speed * 0.7 ) + ( left_speed * 0.3 );

  float pwm_right;

  float pwm_left;

  float demand_left = 0.23; // 1.61 encoder counts per ms for 80
  float demand_right = 0.23; // 1.68 encoder counts per ms for 80

  // ( measurement, demand_right )
  pwm_left = spd_pid_left_class.update( left_speed, demand_left );
  pwm_right = spd_pid_right_class.update( right_speed, demand_right );

  Motors_class.setMotorPower( pwm_left, pwm_right );

}

struct Demand {
  volatile float left;
  volatile float right;
};

struct Demand PID_Controller_left(unsigned long elapsed_time_2, float demand_left, float demand_right) {
  long diff_e_left;
  float left_speed;

  long diff_e_right;
  float right_speed;

  diff_e_left = count_e_left - count_left_last;
  count_left_last = count_e_left;

  diff_e_right = count_e_right - count_right_last;
  count_right_last  = count_e_right;

  left_speed = (float)diff_e_left;
  left_speed /= (float)elapsed_time_2;

  right_speed = (float)diff_e_right;
  right_speed /= (float)elapsed_time_2;

  avg_right_speed = ( avg_right_speed * 0.7 ) + ( right_speed * 0.3 );

  avg_left_speed = ( avg_left_speed * 0.7 ) + ( left_speed * 0.3 );

  float pwm_right;

  float pwm_left;

  struct Demand PWM;

  float bias_demand_left = 0.35; // 1.61 encoder counts per ms for 80
  float bias_demand_right = 0.35; // 1.68 encoder counts per ms for 80

  demand_left = bias_demand_left + demand_left;
  demand_right = bias_demand_right + demand_right;

  // ( measurement, demand_right )
  PWM.left = spd_pid_left_class.update( left_speed, demand_left );
  PWM.right = spd_pid_right_class.update( right_speed, demand_right );

  //  pwm_left = spd_pid_left_class.update( left_speed, demand_left );
  //  pwm_right = spd_pid_right_class.update( right_speed, demand_right );
  //  Motors_class.setMotorPower( pwm_left, pwm_right );

  return PWM;



}

struct Demand PID_Controller_right(unsigned long elapsed_time_2, float demand_left, float demand_right) {
  long diff_e_left;
  float left_speed;

  long diff_e_right;
  float right_speed;

  diff_e_left = count_e_left - count_left_last;
  count_left_last = count_e_left;

  diff_e_right = count_e_right - count_right_last;
  count_right_last  = count_e_right;

  left_speed = (float)diff_e_left;
  left_speed /= (float)elapsed_time_2;

  right_speed = (float)diff_e_right;
  right_speed /= (float)elapsed_time_2;

  avg_right_speed = ( avg_right_speed * 0.7 ) + ( right_speed * 0.3 );

  avg_left_speed = ( avg_left_speed * 0.7 ) + ( left_speed * 0.3 );

  float pwm_right;

  float pwm_left;

  struct Demand PWM;

  float bias_demand_left = 0.35; // 1.61 encoder counts per ms for 80
  float bias_demand_right = 0.35; // 1.68 encoder counts per ms for 80

  // Define a structure to hold sum and product
  demand_left = bias_demand_left + demand_left;
  demand_right = bias_demand_right - demand_right;

  // ( measurement, demand_right )
  //  pwm_left = spd_pid_left_class.update( left_speed, demand_left );
  //  pwm_right = spd_pid_right_class.update( right_speed, demand_right );
  //  Motors_class.setMotorPower( pwm_left, pwm_right );

  PWM.left = spd_pid_left_class.update( left_speed, demand_left );
  PWM.right = spd_pid_right_class.update( right_speed, demand_right );

  return PWM;

}



void turnXDegrees( float angle) {

  float delta_theta_counts = angle * encoderCountperDegree;

  float delta_theta = angle * (PI / 180);
  long count_e_left_store = count_e_left;

  float theta_store = Kinematics_class.theta;

  if ( angle > 0 ) {
    while ( Kinematics_class.theta < ( theta_store + delta_theta ) ) {

      Motors_class.TurnRight( 15, 15 );
      Kinematics_class.update(count_e_left, count_e_right);

    }
  } else {
    while ( Kinematics_class.theta > ( theta_store + delta_theta ) ) {

      Motors_class.TurnLeft( 15, 15 );
      Kinematics_class.update(count_e_left, count_e_right);

    }
  }

  Motors_class.setMotorPower( 0, 0 );
  //  check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);
  //  Serial.print(", ");
  //  Serial.println((theta_store + delta_theta) * (180 / PI));

}


// put your main code here, to run repeatedly:
void loop() {

  unsigned long elapsed_time = millis() - time_prev;
  unsigned long PERIOD = 20; // in microseconds( 1e-6 seconds)
//  Kinematics_class.update(count_e_left, count_e_right);


  //  state = GO_HOME;
  //  Serial.print(", first: ");
  //  Serial.println(state);
  //  Kinematics_class.theta = 50 * (PI / 180);


  if (elapsed_time > PERIOD) {

    //    time_prev = millis();

    while ( ( sqrt( pow(Kinematics_class.X_pos, 2) + pow(Kinematics_class.Y_pos, 2) ) < 250 ) and ( begin_state != DONE ) ) {

      do_PID( elapsed_time );
      time_prev = millis();
      //      Motors_class.setMotorPower( 15, 15 );

      Kinematics_class.update(count_e_left, count_e_right);
      //      check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);

      if ( sqrt( pow(Kinematics_class.X_pos, 2) + pow(Kinematics_class.Y_pos, 2) ) > 200 ) {

        state = Statechecker_class.UpdateState( state, Kinematics_class.X_pos, Kinematics_class.Y_pos, elapsed_time );

        if ( state == ON_LINE ) {

          Statechecker_class.UpdateAction( state, elapsed_time );
          //          PID_Controller( elapsed_time, Linesensor_class.weight_ls_left, Linesensor_class.weight_ls_left);

          begin_state = DONE;

          break;

        }
      }

      elapsed_time = millis() - time_prev;
    }

    state = Statechecker_class.UpdateState( state, Kinematics_class.X_pos, Kinematics_class.Y_pos, elapsed_time );

    Kinematics_class.update(count_e_left, count_e_right);
    //    check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);

    //    Kinematics_class.X_pos = 1000;
    //    Kinematics_class.Y_pos = 1000;
    //    Kinematics_class.theta = 50 * (PI / 180);

    Statechecker_class.UpdateAction( state, elapsed_time );


    if ( sqrt( pow(Kinematics_class.X_pos, 2) + pow(Kinematics_class.Y_pos, 2) ) > 1100 ) {
      //    if ( state == GO_HOME ) {

      if ( state == ON_LINE ) {

        Statechecker_class.UpdateAction( state, elapsed_time );
        //        PID_Controller_left( elapsed_time, Linesensor_class.weight_ls_left, Linesensor_class.weight_ls_left);

        //        Motors_class.setMotorPower( pwm_left, pwm_right );



      } else if ( state == OFF_LINE ) {

        state = GO_HOME;

        Kinematics_class.update(count_e_left, count_e_right);
        Kinematics_class.update(count_e_left, count_e_right);
        //
        if ( Kinematics_class.angle < 0 ) {

          Kinematics_class.angle += (2 * PI);

        }

        float delta_angle_1 = Kinematics_class.angle - Kinematics_class.theta;
        float delta_angle_2 = ( 2 * PI ) - Kinematics_class.angle + Kinematics_class.theta;


        if ( delta_angle_1 > delta_angle_2 ) {

          turnXDegrees( - delta_angle_2 * (180 / PI) ) ;

        } else {

          turnXDegrees(  delta_angle_1 * (180 / PI) ) ;

        }

        //        Motors_class.setMotorPower( 0, 0 );

        //        Serial.println(Kinematics_class.angle);

        //        Motors_class.TurnRight( 15, 15 );


        //        while ( ( sqrt( pow(Kinematics_class.X_pos, 2) + pow(Kinematics_class.Y_pos, 2) ) > 0 ) ) {
        //          do_PID( elapsed_time );
        //          time_prev = millis();
        //          Kinematics_class.update(count_e_left, count_e_right);
        //          elapsed_time = millis() - time_prev;
        //
        //        }




      }

      //      if ( state == GO_HOME) {
      //
      //        if (done_before == false ) {
      //          for (int i = 0; i < 1; i++) {
      //
      //            Kinematics_class.update(count_e_left, count_e_right);
      //
      //            if ( Kinematics_class.angle < 0 ) {
      //
      //              Kinematics_class.angle += (2 * PI);
      //
      //            }
      //
      //            float delta_angle_1 = Kinematics_class.angle - Kinematics_class.theta;
      //            float delta_angle_2 = ( 2 * PI ) - Kinematics_class.angle + Kinematics_class.theta;
      //
      //            Serial.print(delta_angle_1);
      //            Serial.print(", ");
      //            Serial.print(delta_angle_2);
      //            Serial.print(", ");
      //
      //            Serial.print(Kinematics.angle);
      //            Serial.print(", ");
      //            Serial.println(Kinematics.theta);
      //
      //            if ( delta_angle_1 > delta_angle_2 ) {
      //              turnXDegrees( - delta_angle_2 * (180 / PI) ) ;
      //            } else {
      //              turnXDegrees(  delta_angle_1 * (180 / PI) ) ;
      //
      //            }
      //            //            turnXDegrees(  50 ) ;
      //
      //          }
      //          done_before == true;
      //
      //        }
      //
      //      }

    }

    prev_state_1 = state;

    Serial.print(state);

  }
}
