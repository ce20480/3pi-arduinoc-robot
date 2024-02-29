// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "encoders.h"

#define PI 3.14159265359

const float ROBOT_RADIUS_CM = 6.3F; // distance from the centre of robot to wheel 4.5F in cm
const float ROBOT_RADIUS_MM = 82.4F; // distance from the centre of robot to wheel 45F in mm
const float ROBOT_RADIUS_M = 0.096F; // distance in m
const float ROBOT_L_DISTANCE_M = 0.0894F;
const float WHEEL_RADIUS_MM = 1.6F;
const float WHEEL_RADIUS_M = 0.0016F;
const float DISTANCE_PER_COUNT_CM = 0.028057763F; // distance moved in one count in cm
const float DISTANCE_PER_COUNT_MM = 2.8057763F;
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_CIRCUMFERENCE_MM = 10.0530964915F;
const float WHEEL_CIRCUMFERENCE_M = 0.0100530965F;
const float COUNTS_PER_REVOLUTION = 358.32F;
const float DISTANCE_PER_COUNT_M = WHEEL_CIRCUMFERENCE_M / COUNTS_PER_REVOLUTION;
const float ANGULAR_DISPLACEMENT_PER_COUNT = 1.0046885466F;

// General facts
// encoder 358.3 counts per revolution
// 1.0047446274 degrees angular resolution
// 3.2cm wheel diameter, 9cm robot diameter
// 10.0530964915cm is the full distance covered in a revolution
// 0.028057763cm per count

// Class to track robot position.
class Kinematics_c {
  public:
    volatile float X_pos = 0.0F;
    volatile float Y_pos = 0.0F;
    volatile float theta = 0.0F;

    unsigned long elapsed_time_3;

    // Constructor, must exist.
    Kinematics_c() {

    }

    void initialise() {
      setupEncoder0();
      setupEncoder1();
    }

    // Define a structure to hold sum and product
    struct Distance {
      volatile float left;
      volatile float right;
      volatile float X_r_der;
      volatile float theta_r_der;
    };

    // Use this function to update
    // your kinematics

    struct Distance find_distance(long diff_count_left, long diff_count_right) {
      struct Distance distance;
      distance.left = diff_count_left * DISTANCE_PER_COUNT_MM;
      distance.right = diff_count_right * DISTANCE_PER_COUNT_MM;
      distance.X_r_der = (distance.left + distance.right) / 2;
      distance.theta_r_der = ((distance.left - distance.right) / ROBOT_RADIUS_CM) * (PI / 180);
      return distance;
    }

    void update(long diff_count_left, long diff_count_right) {

      unsigned long time_s = micros();

      struct Distance distance;

      // find distances for updating variables
      distance = find_distance(diff_count_left, diff_count_right);

      // Update all the variables
      X_pos = X_pos + distance.X_r_der * cos(theta);
      Y_pos = Y_pos + distance.X_r_der * sin(theta);
      theta = theta + distance.theta_r_der;

      unsigned long elapsed_time_3 = micros() - time_s;


    }


};



#endif
