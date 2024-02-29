#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "statechecker.h"

#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
// of the LED, and toggle it.

int state;
unsigned long time_prev = 0; // timestamp
volatile float left_angular_speed;
volatile float right_angular_speed;
float avg_left_speed; // low pass filter of speed
long count_e_left_last; // for the difference in encoder counts

Motors_c Motors_class;
LineSensor_c Linesensor_class;
StateChecker_c Statechecker_class;
Kinematics_c Kinematics_class;
PID_c spd_pid_left;
PID_c spd_pid_right;

// put your setup code here, to run once:
void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  // Set LED pin as an output
  pinMode( LED_PIN, OUTPUT );

  // Setup encoders when the robot is
  // powered on.
  //  setupEncoder0();
  //  setupEncoder1();
  Kinematics_class.initialise();
  Linesensor_class.initialise_digital();
  Motors_class.initialise();

  // Set initial state of the LED
  led_state = false;
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

// put your main code here, to run repeatedly:
void loop() {

  // Using an if statement to toggle a variable
  //   with each call of loop()
  if ( led_state == true ) {
    led_state = false;
  } else {
    led_state = true;
  }

  // We use the variable to set the
  // debug led on or off on the 3Pi+
  digitalWrite( LED_PIN, led_state );

  Serial.println("loop");
  delay(500);

  unsigned long time_current = micros();
  unsigned long elapsed_time = time_current - time_prev;
  unsigned long PERIOD = 10; // in microseconds( 1e-6 seconds)

  if (elapsed_time > PERIOD) {
    // checking kinematics is working
    //    Kinematics_class.update(count_e_left, count_e_right);
    //    check_kinematics(Kinematics_class.X_pos, Kinematics_class.Y_pos, Kinematics_class.theta);
    //    check_counts();

    // checking accuracy of kinematics
    float left_speed = 20;
    float right_speed = 20;
    float check_angle = 90; // put check angle in degrees
    float check_distance = 500;

    //  check_X_pos( left_speed, right_speed, check_distance);
    check_theta( left_speed, right_speed, check_angle);

    time_prev = time_current;

  }

}
