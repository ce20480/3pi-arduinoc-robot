// this #ifndef stops this file from being included more than once by the compiler
#ifndef _MOTOR_H // checks if _LINESENSOR_H
# define _MOTOR_H
# define L_PWM_PIN1 10
# define L_DIR_PIN1 16
# define R_PWM_PIN1 9
# define R_DIR_PIN1 15

# define FWD LOW
# define REV HIGH

class Motors_c {

    // This keyword means that all code
    // below this point is accesible from
    // outside the class.
  public:

    // These variables are "global" within the
    // class, meaning any part of the class can
    // access them.  Note, they are not global
    // from elsewhere in your Arduino program.
    // These variables will be persistent, and
    // so keep their assigned value between
    // class access operations (with the .)
    int my_int;
    float my_float;
    // https://www.pololu.com/docs/0J83/5.9

    // Constructor, it always needs to exist
    Motors_c() {

    }

    // Functions must be defined in the global
    // scope of the class.  E.g. you cannot
    // create a function inside a function.
    void initialise() {
      pinMode(L_PWM_PIN1, OUTPUT);
      pinMode(L_DIR_PIN1, OUTPUT);
      pinMode(R_PWM_PIN1, OUTPUT);
      pinMode(R_DIR_PIN1, OUTPUT);

      //Start a serial connection
      Serial.begin(9600);

      // Wait for a stable connection, report reset.
      delay(1000);
      Serial.println("***RESET***");

    }

    void setMotorPower(float left_speed, float right_speed) {
       // Set power for motors
      if (left_speed < 0 and right_speed < 0) {
        digitalWrite(L_DIR_PIN1, REV);
        left_speed = abs(left_speed);
        digitalWrite(R_DIR_PIN1, REV);
        right_speed = abs(right_speed); // this is for when the averaging creates negative speeds 
      }
      
      else if (right_speed < 0) {
        digitalWrite(R_DIR_PIN1, REV);
        right_speed = abs(right_speed);
      }
      else if (left_speed < 0) {
        digitalWrite(L_DIR_PIN1, REV);
        left_speed = abs(left_speed);
      }
      else {
        digitalWrite(L_DIR_PIN1, FWD);
        digitalWrite(R_DIR_PIN1, FWD);
      }
      analogWrite(L_PWM_PIN1, left_speed);
      analogWrite(R_PWM_PIN1, right_speed);
    }

    // turn right
    void TurnRight(float left_speed, float right_speed) {
      // Set initial direction (HIGH/LOW)
      // for the direction pins.
      digitalWrite(L_DIR_PIN1, FWD);
      digitalWrite(R_DIR_PIN1, REV);
      analogWrite(L_PWM_PIN1, left_speed);
      analogWrite(R_PWM_PIN1, right_speed);
    }

    // turn left
    void TurnLeft(float left_speed, float right_speed) {
      // Set initial direction (HIGH/LOW)
      // for the direction pins.
      digitalWrite(L_DIR_PIN1, REV);
      digitalWrite(R_DIR_PIN1, FWD);
      analogWrite(L_PWM_PIN1, left_speed);
      analogWrite(R_PWM_PIN1, right_speed);
    }

    // move forward
    void MoveForward(float left_speed, float right_speed) {
      // Set initial direction (HIGH/LOW)
      // for the direction pins.
      digitalWrite(L_DIR_PIN1, FWD);
      digitalWrite(R_DIR_PIN1, FWD);
      analogWrite(L_PWM_PIN1, left_speed);
      analogWrite(R_PWM_PIN1, right_speed);
    }

    // turn right
    void MoveReverse(float left_speed, float right_speed) {
      // Set initial direction (HIGH/LOW)
      // for the direction pins.
      digitalWrite(L_DIR_PIN1, REV);
      digitalWrite(R_DIR_PIN1, REV);
      analogWrite(L_PWM_PIN1, left_speed);
      analogWrite(R_PWM_PIN1, right_speed);
    }

}; // This curly-brace finishes the content of our class.

#endif
