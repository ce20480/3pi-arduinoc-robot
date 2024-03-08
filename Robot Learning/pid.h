#ifndef _PID_H
#define _PID_H

// Class to assign PID controllers
class PID_c {
  public:

    float last_error;
    float p_term;
    float i_term;
    float d_term;
    float i_sum;
    float feedback;

    float p_gain;
    float i_gain;
    float d_gain;

    unsigned long last_timestep;


    PID_c() {

    }

    void init(float kp, float ki, float kd) {

      feedback = 0;
      last_error = 0;
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;

      p_gain = kp;
      i_gain = ki;
      d_gain = kd;

      last_timestep = millis();

    }

    void reset() {

      feedback = 0;
      last_error = 0;
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;

      last_timestep = millis();

    }

    float update( float demand, float measurement ) {

      float error;
      unsigned long current_timestep;
      unsigned long dt;
      float float_dt;
      float diff_error;

      current_timestep = millis();
      dt = current_timestep - last_timestep;

      last_timestep = millis();

      float_dt = (float)dt;

      if (float_dt == 0) return feedback;

      error = measurement - demand;

      p_term = p_gain * error;

      i_sum = i_sum + (error * float_dt);

      i_term = i_gain * i_sum;

      diff_error = (error - last_error) / float_dt;
      last_error = error;
      d_term = diff_error * d_gain;

      feedback = p_term + i_term + d_term;

      return feedback;
    }

};

#endif
