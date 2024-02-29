#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "statechecker.h"
#include "timer3.h"

int state;
unsigned long update_ts = 0; // timestamp

float avg_left_speed; // low pass filter of speed
long count_left_last; // for the difference in encoder counts

float avg_right_speed; // low pass filter of speed
long count_right_last; // for the difference in encoder counts

Motors_c Motors_class;
LineSensor_c Linesensor_class;
StateChecker_c Statechecker_class;
Kinematics_c Kinematics_class;
PID_c spd_pid_left_class;
PID_c spd_pid_right_class;

// put your setup code here, to run once:
// Remember: Setup only runs once when the arduino is powered up.
void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  // Setup encoders and timer interrupts when the robot is
  // powered on and initialise all pins
  Kinematics_class.initialise();
  Linesensor_class.initialise_digital();
  Motors_class.initialise();

  //  setupTimer3(); // f... this function

  // setup kp, ki, kd ( gains )
  //  spd_pid_left_class.initialise( 95.0, 0.7, 0.0 );
  //  spd_pid_right_class.initialise( 90.0, 0.5, 0.0 );
  spd_pid_left_class.initialise( 135.0, 0.5, 0.0 );
  spd_pid_right_class.initialise( 135.0, 0.5, 0.0 );


  update_ts = millis();

  // Set initial state of encoder
  count_left_last = count_e_left;

  // Set initial values for motor
  Motors_class.setMotorPower( 0, 0 ); // for right: BiasPWM 15 is 0.235 demand MaxTurnPWM 60 is 1.23
  // for left BiasPWM 15 is 0.23 demand MaxTurnPWM 60 is 1.19

  // Reset PID before we use it.
  spd_pid_left_class.reset();
  spd_pid_right_class.reset();

}

void do_PID(unsigned long elapsed_time_2, float demand_right, float demand_left) {
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

  //  demand_left = 1.61; // 1.61 encoder counts per ms for 80
  //  demand_right = 1.68; // 1.68 encoder counts per ms for 80

  // ( measurement, demand_right )
  pwm_left = spd_pid_left_class.update( left_speed, demand_left );
  pwm_right = spd_pid_right_class.update( right_speed, demand_right );

  Motors_class.setMotorPower( pwm_left, pwm_right );
}

// put your main code here, to run repeatedly:
void loop() {

  unsigned long elapsed_time_2 = millis() - update_ts;
  unsigned long PERIOD = 30; // in microseconds( 1e-6 seconds) but actually in milliseconds(1e-4 seconds)

  if (elapsed_time_2 > PERIOD) {

    update_ts = millis();

    Linesensor_class.ParallelDigitalReadLineSensors();
    //    LineSensor_c::PIDValues values = Linesensor_class.DigitalLineFollowingLeftPID();
    LineSensor_c::PIDValues values = Linesensor_class.DigitalLineFollowingRightPID();
    float leftDemand = values.leftDemand;
    float rightDemand = values.rightDemand;
    do_PID(elapsed_time_2, leftDemand, rightDemand);
    //    float leftDemand = 0.23;
    //    float rightDemand = 0.23;
    //    do_PID(elapsed_time_2, leftDemand, rightDemand);

  }
}
