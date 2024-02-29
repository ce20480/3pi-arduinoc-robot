// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "encoders.h"

#define PI 3.14159265359

const float ROBOT_RADIUS_CM = 7.2F; // distance from the centre of robot to wheel 4.5F in cm
const float ROBOT_RADIUS_MM = 45.0F; // distance from the centre of robot to wheel 45F in mm
const float WHEEL_RADIUS = 1.6F;
const float DISTANCE_PER_COUNT_CM = 0.028057763F; // distance moved in one count in cm
const float DISTANCE_PER_COUNT_MM = 2.8057763F;
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_CIRCUMFERENCE = 10.0530964915F;
const float COUNTS_PER_REVOLUTION = 358.32F;
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
    volatile long count_left_pre = 0;
    volatile long count_right_pre = 0;

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

    struct Distance find_distance(volatile long count_left, volatile long count_right) {
      struct Distance distance;
      distance.left = (count_left - count_left_pre) * DISTANCE_PER_COUNT_MM;
      distance.right = (count_right - count_right_pre) * DISTANCE_PER_COUNT_MM;
      distance.X_r_der = (distance.left + distance.right) / 2;
      distance.theta_r_der = (distance.left - distance.right) / (2 * ROBOT_RADIUS_CM);
      return distance;
    }

    void update(volatile long count_left, volatile long count_right) {

      struct Distance distance;

      // find distances for updating variables
      distance = find_distance(count_left, count_right);

      // Update all the variables
      X_pos = X_pos + distance.X_r_der * cos(theta);
      Y_pos = Y_pos + distance.X_r_der * sin(theta);
      theta = theta + (distance.theta_r_der * (PI / 180));

      // Update previous count stores and time
      count_left_pre = count_left;
      count_right_pre = count_right;

    }


};



#endif
