// this #ifndef stops this file from being included more than once by the compiler
#ifndef _LINESENSOR_H // checks if _LINESENSOR_H
# define _LINESENSOR_H

# include "motors.h"

# define EMIT_PIN                   11   // Documentation says 11.
# define LS_LEFTMOST_PIN            12   // Complete for DN1 pin
# define LS_LEFT_PIN                18   // Complete for DN2 pin
# define LS_MIDDLE_PIN              20   // Complete for DN3 pin
# define LS_RIGHT_PIN               21   // Complete for DN4 pin
# define LS_RIGHTMOST_PIN           22   // Complete for DN5 pin

# define NUM_LS_PINS                 3   // How many pins are going to be read

# define ANALOGUE_LS_LEFTMOST_PIN  A11   // Pin for DN1 for analogue analysis
# define ANALOGUE_LS_LEFT_PIN       A0   // Pin for DN2 for analogue analysis
# define ANALOGUE_LS_MIDDLE_PIN     A2   // Pin for DN3 for analogue analysis
# define ANALOGUE_LS_RIGHT_PIN      A3   // Pin for DN4 for analogue analysis
# define ANALOGUE_LS_RIGHTMOST_PIN  A4   // Pin for DN5 for analogue analysis

# define BiasPWM                    20   // Bias parameter for motors, changed for when I have cable attached
# define MaxTurnPWM                 54   // Max speed for turning

Motors_c motors;

// Class to operate the linesensor(s).
class LineSensor_c {

  public:

    int ls_pins[ NUM_LS_PINS ] = { LS_LEFT_PIN, LS_MIDDLE_PIN, LS_RIGHT_PIN }; // always match the entries to the max number of pins decided before, count entries from 0 in arrays
    float ls_reading[ NUM_LS_PINS ];
    float weight_ls_left;
    float weight_ls_right;

    // Constructor must exist
    LineSensor_c() {
    }

    void initialise_digital() {
      // Set some initial pin modes and states
      //      pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off), Infrared LED on
      pinMode( LS_LEFTMOST_PIN , INPUT );     // Set line sensor pin to input
      pinMode( LS_LEFT_PIN , INPUT );     // Set line sensor pin to input
      pinMode( LS_MIDDLE_PIN , INPUT );     // Set line sensor pin to input
      pinMode( LS_RIGHT_PIN , INPUT );     // Set line sensor pin to input
      pinMode( LS_RIGHTMOST_PIN , INPUT );     // Set line sensor pin to input

      // Start Serial, wait to connect, print a debug message.
      Serial.begin(9600);
      delay(1500);
      Serial.println("***RESET***");
    }

    void initialise_analogue() {
      // Set some initial pin modes and states
      //      pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off), Infrared LED on
      pinMode( ANALOGUE_LS_LEFTMOST_PIN, INPUT_PULLUP );     // Set line sensor pin to input
      pinMode( ANALOGUE_LS_LEFT_PIN , INPUT_PULLUP );     // Set line sensor pin to input
      pinMode( ANALOGUE_LS_MIDDLE_PIN , INPUT_PULLUP );     // Set line sensor pin to input
      pinMode( ANALOGUE_LS_RIGHT_PIN , INPUT_PULLUP );     // Set line sensor pin to input
      pinMode( ANALOGUE_LS_RIGHTMOST_PIN , INPUT_PULLUP );     // Set line sensor pin to input

      // Start Serial, wait to connect, print a debug message.
      Serial.begin(9600);
      delay(1500);
      Serial.println("***RESET***");

    }

    void IREmitON() { // IR-EMIT pin turned on
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH);
    }

    void IREmitOFF() { // IR-EMIT pin turned off
      pinMode( EMIT_PIN, INPUT );
    }

    void ChargePin( int which_pin ) { // charge capacitor for intended pin
      // Charge each of the capacitors
      pinMode( which_pin, OUTPUT );
      digitalWrite( which_pin, HIGH );
    }

    void DischargePin( int which_pin ) { // start discharging for intended pin
      pinMode ( which_pin, INPUT );
    }

    void ParallelDigitalReadLineSensors() {
      // We will use which to index through sensors
      int which;

      // We will count how many sensors have discharged
      int count;

      // record start time
      unsigned long start_time;
      start_time = micros();

      IREmitON();

      // setup end times ( and  can ensure that they are all zero to begin with but less generalisable)
      unsigned long end_times_ls[ NUM_LS_PINS ];

      for ( which = 0; which < NUM_LS_PINS; which++ ) {

        // charge each of the capacitors
        ChargePin( ls_pins[ which ] );


        end_times_ls[ which ] = 0;

      }

      // Allow time to charge
      delayMicroseconds(10);

      //Set each of the capacitors to on so they start losing charge
      for ( which = 0; which < NUM_LS_PINS; which++ ) {
        //        pinMode( ls_pins[ which ], INPUT );
        DischargePin( ls_pins[ which ] );
      }

      // set checking parameter
      bool done = false;

      //Start count at number of pins and then count down
      count = NUM_LS_PINS;

      // loop until all 3 sensors are read( discharged)
      while ( done == false ) {
        for ( which = 0; which < NUM_LS_PINS; which++ ) {

          // update end time
          if ( end_times_ls[ which ] == 0 ) {

            if ( digitalRead( ls_pins[ which ] ) == LOW ) {

              end_times_ls[ which ] = micros();

              // update count
              count = count - 1;

            } // check which sensors are low and update end times

          } // check if sensors has already been read

          if (count == 0)  {
            done = true;

          } // if all sensors are read

        } // iterate through all sensors to check end times

      } // keep iterating each tick until all sensors have been checked

      // save reading for each sensor
      for ( which = 0; which < NUM_LS_PINS; which++ ) {
        unsigned long elapsed_time;
        elapsed_time = end_times_ls[ which ] - start_time;
        ls_reading[ which ] = (float)elapsed_time;
      }

      IREmitOFF();

      int sum;

      float n_list[ NUM_LS_PINS ];

      for ( int i = 0; i < NUM_LS_PINS; i++ ) {
        if ( i != 1 ) {
          sum += ls_reading[ i ]; // hard coding to ignore middle sensor
        }
      }

      for ( int i = 0; i < NUM_LS_PINS; i++ ) {
        if ( i != 1 ) {
          n_list[i] = ls_reading[ i ] / sum;
          n_list[i] = n_list[i] * 2;
        }
      }

      weight_ls_left = n_list[0] - n_list[2]; // since its left pin minus right pin if weight is positive then need to turn left
      weight_ls_right = n_list[2] - n_list[0];

    }

    unsigned long FindElapsedTime() {

      unsigned long elapsed_time;
      unsigned long start_time;
      unsigned long end_time;

      start_time = micros();

      ParallelDigitalReadLineSensors();

      end_time = micros();

      elapsed_time = end_time - start_time;

      return elapsed_time;

    }

    void DigitalLineFollowingLeft() {

      float LeftPWM;
      float RightPWM;

      LeftPWM = BiasPWM + (weight_ls_right * MaxTurnPWM);
      RightPWM = BiasPWM - (weight_ls_right * MaxTurnPWM);
      motors.setMotorPower( LeftPWM, RightPWM );

      //      unsigned long elapsed_time;
      //
      //      elapsed_time = FindElapsedTime();
      //
      //      if ( elapsed_time >= 1500 ) {
      //        LeftPWM = BiasPWM + (weight_ls_right * MaxTurnPWM);
      //        RightPWM = BiasPWM - (weight_ls_right * MaxTurnPWM);
      //        motors.setMotorPower( LeftPWM, RightPWM );
      //      } else {
      //        motors.setMotorPower( 0, 0 );
      //      }
    }

    void DigitalLineFollowingRight() {

      float LeftPWM;
      float RightPWM;

      LeftPWM = BiasPWM - (weight_ls_left * MaxTurnPWM);
      RightPWM = BiasPWM + (weight_ls_left * MaxTurnPWM);
      motors.setMotorPower( LeftPWM, RightPWM );
      //
      //      unsigned long elapsed_time;
      //
      //      elapsed_time = FindElapsedTime();
      //
      //      if ( elapsed_time >= 1500 ) {
      //        LeftPWM = BiasPWM - (weight_ls_left * MaxTurnPWM);
      //        RightPWM = BiasPWM + (weight_ls_left * MaxTurnPWM);
      //        motors.setMotorPower( LeftPWM, RightPWM );
      //      } else {
      //        motors.setMotorPower( 0, 0 );
      //      }
    }

    unsigned long SeriesDigitalReadLineSensors( int which_pin) {
      // Complete the steps referring to the pseudocode block
      // Algorithm 1.
      // The first steps have been done for you.
      // Fix parts labelled ????
      // Some steps are missing - add these.
      IREmitON();

      // Step 1: charging the capacitor for the pin
      //      pinMode( which_pin, OUTPUT );
      //      digitalWrite( which_pin, HIGH );
      ChargePin( which_pin );
      delayMicroseconds( 10 );
      DischargePin( which_pin );

      //      pinMode( which_pin, INPUT);
      unsigned long start_time = micros();
      unsigned long break_time = 5000;

      while ( digitalRead( which_pin ) == HIGH ) {
        // Do nothing here (waiting).
        unsigned long end_time = micros();
        unsigned long elapsed_time = end_time - start_time;
        if (elapsed_time > break_time) {
          break;
        }
      }

      unsigned long end_time = micros();

      IREmitOFF();

      unsigned long elapsed_time = end_time - start_time;

      return elapsed_time;
    }

    unsigned long ReadRightmostSensor() {
      IREmitON();
      // Step 1: charging the capacitor for the pin
      //      pinMode( which_pin, OUTPUT );
      //      digitalWrite( which_pin, HIGH );
      ChargePin( LS_RIGHTMOST_PIN );
      delayMicroseconds( 10 );
      DischargePin( LS_RIGHTMOST_PIN );

      //      pinMode( which_pin, INPUT);
      unsigned long start_time = micros();
      unsigned long break_time = 5000;

      while ( digitalRead( LS_RIGHTMOST_PIN ) == HIGH ) {
        // Do nothing here (waiting).
        unsigned long end_time = micros();
        unsigned long elapsed_time = end_time - start_time;
        if (elapsed_time > break_time) {
          break;
        }
      }

      unsigned long end_time = micros();

      IREmitOFF();

      unsigned long elapsed_time = end_time - start_time;

      return elapsed_time;
    }
    
    unsigned long ReadMiddleSensor() {
      IREmitON();
      // Step 1: charging the capacitor for the pin
      //      pinMode( which_pin, OUTPUT );
      //      digitalWrite( which_pin, HIGH );
      ChargePin( LS_MIDDLE_PIN );
      delayMicroseconds( 10 );
      DischargePin( LS_MIDDLE_PIN );

      //      pinMode( which_pin, INPUT);
      unsigned long start_time = micros();
      unsigned long break_time = 5000;

      while ( digitalRead( LS_RIGHTMOST_PIN ) == HIGH ) {
        // Do nothing here (waiting).
        unsigned long end_time = micros();
        unsigned long elapsed_time = end_time - start_time;
        if (elapsed_time > break_time) {
          break;
        }
      }

      unsigned long end_time = micros();

      IREmitOFF();

      unsigned long elapsed_time = end_time - start_time;

      return elapsed_time;
    }

};

#endif
