#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "statechecker.h"
#include "timer3.h"

int state;
unsigned long update_ts = 0; // timestamp

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

  setupTimer3();

  // setup kp, ki, kd ( gains )
  //  spd_pid_left_class.initialise( 95.0, 0.7, 0.0 );
  //  spd_pid_right_class.initialise( 90.0, 0.5, 0.0 );


  //  update_ts = millis();

  // Set initial state of encoder
  //  count_left_last = count_e_left;

  // Set initial values for motor
  //  Motors_class.setMotorPower( 0, 0 );

  // Reset PID before we use it.
  //  spd_pid_left_class.reset();
  //  spd_pid_right_class.reset();

}


void check_kinematics(volatile float X_pos, volatile float Y_pos, volatile float theta) { // , volatile long diff_left_count, volatile long diff_right_count, volatile long count_left, long count_right) {
  Serial.print(X_pos);
  Serial.print(", ");
  Serial.print(Y_pos);
  Serial.print(", ");
  //  Serial.print(diff_left_count);
  //  Serial.print(", ");
  //  Serial.println(diff_right_count);
  //  Serial.print(", ");
  //  Serial.print(count_left);
  //  Serial.print(", ");
  //  Serial.println(count_right);
  //  Serial.print(", ");
  //  Serial.print(PI);
  float theta_2 = theta * (180 / PI);
  Serial.print(", ");
  Serial.println(theta_2);
  //  Serial.print(" ,");
  //  Serial.println(theta_r_der);

  //  Serial.println(isr_execution_time);
}

void check_theta( float left_speed, float right_speed, float check_angle) { //, volatile long diff_left_count, volatile long diff_right_count, volatile long count_left, long count_right) {

  check_angle = check_angle * (PI / 180);

  while (Kinematics_class.theta < 100000) {

    unsigned long time_s = micros();

    Motors_class.TurnRight( left_speed, right_speed );
    Kinematics_class.update(diff_left_count, diff_right_count);
    unsigned long elapsed_time = micros() - time_s;

    Serial.print(Kinematics_class.X_pos);
    Serial.print(", ");
    Serial.print(Kinematics_class.Y_pos);
    Serial.print(", ");
    float theta_2 = Kinematics_class.theta * (180 / PI);
    Serial.print(", ");
    Serial.print(theta_2);
    Serial.print(", ");
    Serial.print(elapsed_time);
    Serial.print(", ");
    Serial.println(Kinematics_class.elapsed_time_3);
  }

  Motors_class.setMotorPower( 0, 0 );
}

void check_X_pos( float left_speed, float right_speed, float check_distance, volatile long diff_left_count, volatile long diff_right_count) {

  while (Kinematics_class.X_pos < check_distance) {

    unsigned long time_check = micros();

    if ( time_check > isr_execution_time_t3 ) {

      Motors_class.setMotorPower( left_speed, right_speed );
      Kinematics_class.update(diff_left_count, diff_right_count);
      check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta); //, diff_left_count, diff_right_count, count_e_left, count_e_right);

    }

  }

  Motors_class.setMotorPower( 0, 0 );
}

// put your main code here, to run repeatedly:
void loop() {

  unsigned long elapsed_time_2 = millis() - update_ts;
  unsigned long PERIOD = 20; // in microseconds( 1e-6 seconds) but actually in milliseconds(1e-4 seconds)

  //  if (elapsed_time_2 > PERIOD) { // makes speed curves more analogue

  update_ts = millis();

  // checking kinematics is working
  //    Kinematics_class.update(diff_left_count, diff_right_count);
  //    check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta, diff_left_count, diff_right_count);

  //    checking accuracy of kinematics
  float left_speed = 20;
  float right_speed = 20;
  float check_angle = 90; // put check angle in degrees
  float check_distance = 500;

  //    check_X_pos( left_speed, right_speed, check_distance, diff_left_count, diff_right_count);
  //
  //  Serial.print(isr_execution_time_t3);
  //  Serial.print(", ");
  //  Serial.print(diff_left_count);
  //  Serial.print(", ");
  //  Serial.println(diff_right_count);

  check_theta( left_speed, right_speed, check_angle );// diff_left_count, diff_right_count, count_e_left, count_e_right);

  //    Serial.print(count_e_left);
  //    Serial.print(", ");
  //    Serial.println(count_e_right);
  //    //  Serial.print(", ");
  //    //  Serial.print(PI);
  //    //  float theta_2 = theta * (180 / PI);
  //    //  Serial.print(", ");
  //    //  Serial.println(theta_2);
  //    //  Serial.print(" ,");
  //    //  Serial.println(theta_r_der);

  //}
}
