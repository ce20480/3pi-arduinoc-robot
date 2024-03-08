#ifndef _FEATURES_H
#define _FEATURES_H
#define YELLOW LED_BUILTIN
#define RED LED_BUILTIN_RX
#define GREEN LED_BUILTIN_TX
#define BUZZER 6
#define NUM_COLORS 3

// Class to (de)activate robot features including LED's and buzzer
class Features_c {

  public:

    int colors[NUM_COLORS] = {YELLOW, RED, GREEN};

    Features_c() {

    }

    void init() {

      for (int i = 0; i < NUM_COLORS; i++) {
        pinMode(colors[i], OUTPUT);
      }

      pinMode(BUZZER, OUTPUT);

    }

    void buzz(int frequency, int buzz_time) {

      unsigned long start_time = millis();
      bool beep = true;

      while (beep) {
        unsigned long elapsed_time = millis();

        digitalWrite(BUZZER, HIGH);
        delayMicroseconds(frequency);
        digitalWrite(BUZZER, LOW);
        delayMicroseconds(frequency);

        if (elapsed_time - start_time > buzz_time) {
          beep = false;
        }
      }
    }

    void ledOn(int color) {

      if (color == YELLOW) {
        digitalWrite(color, HIGH);
      } else if (color == RED || color == GREEN) {
        digitalWrite(color, LOW);
      } else {
        Serial.println("Invalid color");
        delay(1000);
      }
    }

    void ledOff(int color) {

      if (color == YELLOW) {
        digitalWrite(color, LOW);
      } else if (color == RED || color == GREEN) {
        digitalWrite(color, HIGH);
      } else {
        Serial.println("Invalid color");
        delay(1000);
      }
    }

    void allLedOn() {
      for (int i = 0; i < NUM_COLORS; i++) {
        ledOn(colors[i]);
      }
    }

    void allLedOff() {
      for (int i = 0; i < NUM_COLORS; i++) {
        ledOff(colors[i]);
      }
    }

    void flash(int color, int flash_time) {

      ledOn(color);
      delay(flash_time);
      ledOff(color);
      delay(flash_time);

    }

    void flashAll(int flash_time) {

      for (int i = 0; i < NUM_COLORS; i++) {
        ledOn(colors[i]);
      }

      delay(flash_time);

      for (int i = 0; i < NUM_COLORS; i++) {
        ledOff(colors[i]);
      }

      delay(flash_time);
    }

    void plot3val(float val1, float val2, float val3) {

      Serial.print(val1);
      Serial.print(",");
      Serial.print(val2);
      Serial.print(",");
      Serial.print(val3);
      Serial.print("\n");
    }

};

#endif
