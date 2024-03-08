#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

#define MAXSPEED 255

// Class to operate the motor(s).
class Motors_c {
  public:

    float left_pwm;
    float right_pwm;

    Motors_c() {

    }

    void init() {

      left_pwm = 0;
      right_pwm = 0;

      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);

      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, FWD);
      analogWrite(L_PWM_PIN, left_pwm);
      analogWrite(R_PWM_PIN, right_pwm);

    }

    void setMotorPower( float left, float right ) {

      left_pwm = left;
      right_pwm = right;

      if (abs(left_pwm) < MAXSPEED) {
        if (left_pwm < 0) {
          digitalWrite(L_DIR_PIN, REV);
          analogWrite(L_PWM_PIN, abs(left_pwm));
        }
        else {
          digitalWrite(L_DIR_PIN, FWD);
          analogWrite(L_PWM_PIN, left_pwm);
        }
      }
      else {
        if (left_pwm < 0) {
          digitalWrite(L_DIR_PIN, REV);
          analogWrite(L_PWM_PIN, MAXSPEED);
        }
        else {
          digitalWrite(L_DIR_PIN, FWD);
          analogWrite(L_PWM_PIN, MAXSPEED);
        }
      }

      if (abs(right_pwm) < MAXSPEED) {
        if (right_pwm < 0) {
          digitalWrite(R_DIR_PIN, REV);
          analogWrite(R_PWM_PIN, abs(right_pwm));
        } else {
          digitalWrite(R_DIR_PIN, FWD);
          analogWrite(R_PWM_PIN, right_pwm);
        }
      }
      else {
        if (right_pwm < 0) {
          digitalWrite(R_DIR_PIN, REV);
          analogWrite(R_PWM_PIN, MAXSPEED);
        }
        else {
          digitalWrite(R_DIR_PIN, FWD);
          analogWrite(R_PWM_PIN, MAXSPEED);
        }
      }
    }

    float getLeftMotorPower() {
      return left_pwm;
    }

    float getRightMotorPower() {
      return right_pwm;
    }


};

#endif
