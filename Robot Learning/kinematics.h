#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#define ROBOT_DIAMETER 90
#define WHEEL_RADIUS 17
#include <math.h>

// Class to track robot position.
class Kinematics_c {
  public:

    float r = WHEEL_RADIUS;
    float l = ROBOT_DIAMETER / 2;
    float cpr = 358.3;
    float x_current = 0;
    float y_current = 0;
    float theta_current = 0;
    float x_delta;
    float theta_delta;
    float vel_left;
    float vel_right;
    long encode_left_old;
    long encode_right_old;
    int rotations = 0;

    Kinematics_c() {

    }

    void init(long encode_left, long encode_right) {

      encode_left_old = encode_left;
      encode_right_old = encode_right;

    }

    void update(long encode_left, long encode_right) {

      float x_new;
      float y_new;
      float theta_new;

      vel_left = encode_left_old - encode_left;
      vel_right = encode_right_old - encode_right;

      encode_left_old = encode_left;
      encode_right_old = encode_right;

      vel_left *= (2 * M_PI * r) / cpr;
      vel_right *= (2 * M_PI * r) / cpr;

      x_delta = (vel_left / 2) + (vel_right / 2);
      theta_delta = (vel_left / (2 * l)) - (vel_right / (2 * l));

      x_new = x_current + (x_delta * cos(theta_current));
      y_new = y_current + (x_delta * sin(theta_current));
      theta_new = theta_current + theta_delta;

      if (theta_new - (2 * M_PI * rotations) > 2 * M_PI) {
        rotations++;
      } else if (theta_new - (2 * M_PI * rotations) < 0) {
        rotations--;
      }

      x_current = x_new;
      y_current = y_new;
      theta_current = theta_new;
    }

    float degrees_to_radians(float theta_deg) {

      return (theta_deg * M_PI) / 180;
    }

    float radians_to_degrees(float theta_rad) {

      return theta_rad * (180 / M_PI);
    }

};



#endif
