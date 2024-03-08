#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define EMIT_PIN 11
#define LS_LEFT_PIN 12
#define LS_MIDLEFT_PIN 18
#define LS_MIDDLE_PIN 20
#define LS_MIDRIGHT_PIN 21
#define LS_RIGHT_PIN 22

#define LS_LEFT_ADC A11
#define LS_MIDLEFT_ADC A0
#define LS_MIDDLE_ADC A2
#define LS_MIDRIGHT_ADC A3
#define LS_RIGHT_ADC  A4

#define NUM_SENSORS 5
#define LINE_DETECT 860

// Class to operate the linesensor(s).
class LineSensor_c {
  public:

    int ls_pins_adc[NUM_SENSORS] = {LS_LEFT_ADC,
                                    LS_MIDLEFT_ADC,
                                    LS_MIDDLE_ADC,
                                    LS_MIDRIGHT_ADC,
                                    LS_RIGHT_ADC
                                   };
    int sensor_readings[NUM_SENSORS] = {0};
    boolean pins_detected[NUM_SENSORS] = {false};



    LineSensor_c() {
      
      pinMode( EMIT_PIN, OUTPUT);
      digitalWrite( EMIT_PIN, HIGH);
    }


    void init(int pin_num) {
      
      pinMode( ls_pins_adc[pin_num], INPUT_PULLUP);
    }


    void initAll() {

      for (int i = 0; i < NUM_SENSORS; i++) {
        init(i);
      }
    }


    boolean lineDetected( float reading ) {

      if (reading > LINE_DETECT) {
        return true;
      } else {
        return false;
      }
    }

    int readADC( int pin_num ) {

      int reading_adc;

      reading_adc = analogRead(ls_pins_adc[pin_num]);

      return reading_adc;

    }

    void readDetectAll() {

      for (int i = 0; i < NUM_SENSORS; i++) {
        int reading = readADC(i);
        sensor_readings[i] = reading;
        if (lineDetected(reading)) {
          pins_detected[i] = true;
        }
        else {
          pins_detected[i] = false;
        }
      }
    }

    boolean checkDetected(boolean ls_left, boolean ls_left_mid, boolean ls_mid, boolean ls_right_mid, boolean ls_right) {

      boolean check_array[NUM_SENSORS] = {ls_left,
                                          ls_left_mid,
                                          ls_mid,
                                          ls_right_mid,
                                          ls_right
                                         };

      for (int i = 0; i < NUM_SENSORS; i++) {

        if (pins_detected[i] != check_array[i]) {
          return false;
        }
      }

      return true;

    }

    float calculate_weight() {

      float read1 = sensor_readings[1];
      float read2 = sensor_readings[3];

      float sum = read1 + read2;

      float n1 = read1 / sum * 2;
      float n2 = read2 / sum * 2;

      return n1 - n2;
    }
};

#endif
