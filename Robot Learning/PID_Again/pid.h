// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:


    // PID update variables
    float feedback;
    float last_error;
    float P_term;
    float I_term;
    float D_term;
    float I_sum;

    float P_gain;
    float I_gain;
    float D_gain;

    unsigned long ms_last_ts;

    // Constructor, must exist.
    PID_c() {

    }

    void initialise(float gain_proportional, float gain_integral, float gain_derivative) {

      feedback = 0;
      last_error = 0;
      P_term = 0;
      I_term = 0;
      D_term = 0;
      I_sum = 0;

      P_gain = gain_proportional; // current error helps fix current error and shoot quickly towards demand but results in a steady-state equilibrium
      I_gain = gain_integral; // accumulated errors helps fix steady-state errors
      D_gain = gain_derivative; // change in error helps fix large overshoot for when large changes of speeds occur

      ms_last_ts = millis();

    }

    // best to reset PID before next use because delays can build up i-term, example when changing states
    void reset() {

      feedback = 0;
      last_error = 0;
      P_term = 0;
      I_term = 0;
      D_term = 0;
      I_sum = 0;

      ms_last_ts = millis();
    }

    float update( float demand, float measurement ) {

      // create necessary variables
      float error;
      unsigned long ms_now_ts;
      unsigned long ms_dt;
      float float_dt;
      float diff_error;

      // Grab time to calc elapsed time
      ms_now_ts = millis();
      ms_dt = ms_now_ts - ms_last_ts;

      // ms_last_t has been used, so update
      // it for the next call of this update
      ms_last_ts = millis();

      // typecasting  the difference of two unsigned longs is safer
      float_dt = (float)ms_dt;

      // NOTE: A serious error can occur
      // here if dt = 0, this causes divide
      // by zero errors. This can happen if
      // PID.update() is called faster than 1ms.
      // Here, we catch the error by returning
      // the last feedback value
      if ( float_dt == 0 ) return feedback;

      // Calculate error signal
      error = measurement - demand;

      // P term, changing current error
      P_term = P_gain * error;

      // discrete integration, solving for steady-state error
      I_sum = I_sum + ( error * float_dt );

      // I term, accumulation error 
      I_term = I_gain * I_sum;

      // D term, changing error ( quick speed changes error)
      // NOTE: sometimes this needs to be inverted
      diff_error = (error - last_error) / float_dt; // slope of the line
      last_error = error;
      D_term = D_gain * diff_error;

      // the whole thing!
      // Again, sometimes this needs to be -D term
      // D term should counteract sudden changes, which 
      // means it is subtractive. Another way to achieve 
      // this is to have a -ve gain. Best way to trouble
      // shoot is to plot what it is doing
      feedback = P_term + I_term + D_term; 

      // output feedback
      return feedback;

    }

};



#endif
