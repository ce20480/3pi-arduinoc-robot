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
  RIGHT_TURN_STRONG = 3,
  GO_HOME = 4,

  JOIN_LINE = 5,
  RIGHT_TURN_WEAK = 6,
  DONE = 7,



} STATE;

volatile int state = BEGIN;
int gain = 0;


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

  time_prev = micros();
  count_left_last = count_e_left;
  count_right_last = count_e_right;
  unsigned long encoder_time = micros();

  // Setup encoders when the robot is
  // powered on.


  Kinematics_class.initialise();
  Linesensor_class.initialise_digital();
  Motors_class.initialise();
  spd_pid_left_class.initialise( 6, 0.2, 0.0 ); // 8 2 100
  spd_pid_right_class.initialise( 6, 0.2, 0.0 );// 8 2 100

  Motors_class.setMotorPower( 0, 0 );

  // Set initial state of the LED
  led_state = false;
  spd_pid_left_class.reset();
  spd_pid_right_class.reset();

  prev_state = state;


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
  Serial.println(theta_2);
  //  Serial.print(" ,");
  //  Serial.println(theta_r_der);
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

  float demand_left = 0.13; // 1.61 encoder counts per ms for 80
  float demand_right = 0.13; // 1.68 encoder counts per ms for 80

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

  //  unsigned long elapsed_left = millis() - spd_pid_left_class.ms_last_ts;

  diff_e_right = count_e_right - count_right_last;
  count_right_last  = count_e_right;

  //  unsigned long elapsed_right = millis() - spd_pid_right_class.ms_last_ts;


  left_speed = (float)diff_e_left;
  left_speed /= (float)elapsed_time_2;

  right_speed = (float)diff_e_right;
  right_speed /= (float)elapsed_time_2;

  avg_right_speed = ( avg_right_speed * 0.7 ) + ( right_speed * 0.3 );

  avg_left_speed = ( avg_left_speed * 0.7 ) + ( left_speed * 0.3 );

  float pwm_right;

  float pwm_left;

  struct Demand PWM;

  //  float bias_demand_left = 0.26; // 1.61 encoder counts per ms for 80
  //  float bias_demand_right = 0.26; // 1.68 encoder counts per ms for 80
  //
  //  demand_left = bias_demand_left + demand_left;
  //  demand_right = bias_demand_right + demand_right;

  float bias_demand_left = 0.23; // 1.61 encoder counts per ms for 80
  float bias_demand_right = 0.23; // 1.68 encoder counts per ms for 80

  //  demand_left = bias_demand_left - (demand_left / 3);
  //  demand_right = bias_demand_right + (demand_right / 3);


  // ( measurement, demand_right )
  //  PWM.left = spd_pid_left_class.update( left_speed, demand_left );
  //  PWM.right = spd_pid_right_class.update( right_speed, demand_right );
  PWM.left = spd_pid_left_class.update( 0, demand_left );
  PWM.right = spd_pid_right_class.update( 0, demand_right );
  //
  //  PWM.left = ( PWM.left / (PWM.left + PWM.right) ) * 20;
  //  PWM.right = ( PWM.right / (PWM.left + PWM.right) ) * 20;


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

  float bias_demand_left = 0.12; // 1.61 encoder counts per ms for 80
  float bias_demand_right = 0.12; // 1.68 encoder counts per ms for 80

  // Define a structure to hold sum and product
  demand_left = bias_demand_left - (demand_left / 4);
  demand_right = bias_demand_right + (demand_right / 4);

  // ( measurement, demand_right )
  //  pwm_left = spd_pid_left_class.update( left_speed, demand_left );
  //  pwm_right = spd_pid_right_class.update( right_speed, demand_right );
  //  Motors_class.setMotorPower( pwm_left, pwm_right );

  PWM.left = spd_pid_left_class.update( 0, demand_left );
  PWM.right = spd_pid_right_class.update( 0, demand_right );

  return PWM;

}



void turnXDegrees( float angle) {

  float delta_theta_counts = angle * encoderCountperDegree;

  float delta_theta = angle * (PI / 180);
  long count_e_left_store = count_e_left;

  float theta_store = Kinematics_class.theta;

  while ( Kinematics_class.theta < ( theta_store + delta_theta ) ) {

    Motors_class.TurnRight( 15, 15 );
    Kinematics_class.update(count_e_left, count_e_right);

  }

  Motors_class.setMotorPower( 0, 0 );
  //  check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);
  //  Serial.print(", ");
  //  Serial.println((theta_store + delta_theta) * (180 / PI));

}



// Put your main code here, to run repeatedly:
void loop() {

  unsigned long elapsed_time = millis() - time_prev;
  unsigned long PERIOD = 20; // in microseconds( 1e-6 seconds)

  if (elapsed_time > PERIOD) {
    while ( ( sqrt( pow(Kinematics_class.X_pos, 2) + pow(Kinematics_class.Y_pos, 2) ) < 250 ) and ( begin_state != DONE ) ) {

      do_PID( elapsed_time );
      time_prev = millis();

      Kinematics_class.update(count_e_left, count_e_right);
      check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);

      if ( sqrt( pow(Kinematics_class.X_pos, 2) + pow(Kinematics_class.Y_pos, 2) ) > 200 ) {

        state = JOIN_LINE;

        state = Statechecker_class.UpdateState( state, Kinematics_class.X_pos, Kinematics_class.Y_pos, elapsed_time );

        if (state == ON_LINE) {

          spd_pid_left_class.reset();
          spd_pid_right_class.reset();

          begin_state = DONE;

        }

      }
    }


    state = Statechecker_class.UpdateState( state, Kinematics_class.X_pos, Kinematics_class.Y_pos, elapsed_time );

    Kinematics_class.update(count_e_left, count_e_right);

    //    check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);

    if ( state == RIGHT_TURN_STRONG ) {

      turnXDegrees( 45 );

    }

    if ( state == RIGHT_TURN_STRONG ) {
      turnXDegrees( 10 );
    }
    
    //  }

    if ( state == ON_LINE ) {



      unsigned long placeholder = Linesensor_class.FindElapsedTime();


      Linesensor_class.DigitalLineFollowingLeft();


      //      struct Demand PWM;
      //
      //      PWM = PID_Controller_left( elapsed_time, Linesensor_class.weight_ls_left, Linesensor_class.weight_ls_left);
      //      time_prev = millis();
      //
      //      Motors_class.setMotorPower( PWM.left + 20, PWM.right - 20 );
      //
      //      Serial.print(PWM.left);
      //      Serial.print(",");
      //      Serial.print(PWM.right);
      //        Serial.print(",");



    }
    //
    if ( state == OFF_LINE ) {

      Motors.TurnLeft( 15, 15 );

    }

    if ( state != prev_state) {
      spd_pid_left_class.reset();
      spd_pid_right_class.reset();
    }

    prev_state = state;

    //    Serial.println(state);

  }


}
