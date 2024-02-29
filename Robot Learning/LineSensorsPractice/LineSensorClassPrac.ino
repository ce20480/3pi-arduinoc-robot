# include "linesensor.h"
# include "motor.h"

# define NUM_LS_PINS2                2   // How many pins are going to be read
//# define LS_LEFT_PIN                18   // Complete for DN2 pin
//# define LS_RIGHT_PIN               21   // Complete for DN4 pin
# define BiasPWM                    15   // Bias parameter for motors
# define MaxTurnPWM                 60   // Max speed for turning

//int ls_pins2[ NUM_LS_PINS ] = { LS_LEFT_PIN, LS_RIGHT_PIN };
float LeftPWM;
float RightPWM;

LineSensor_c line_sensors;
Motors_c motors;

void setup() {

  line_sensors.initialise_digital();
  line_sensors.initialise_analogue();

} // End of setup()

void loop() {

  unsigned long elapsed_time;
  unsigned long end_time;
  unsigned long start_time;
//  int my_adc_read;
  //  unsigned long ls_reading2[ NUM LS PINS ];
  //  float n_list[ NUM_LS_PINS ];
  float weight_ls_1;
  float weight_ls_2;


  // analogue version for reading
  //  my_adc_read = analogRead( ANALOGUE_LS_MIDDLE_PIN );
  //  Serial.print( my_adc_read );
  //  Serial.print(",");

  //  ls_reading2 = line_sensors.ParallelDigitalReadLineSensors2(ls_pins2, NUM_LS_PINS2);
//  weight_ls_1 = line_sensors.WeightedMeasurementLeft;

  start_time = micros();

  line_sensors.ParallelDigitalReadLineSensors();

  end_time = micros();

  elapsed_time = end_time - start_time;

  if ( elapsed_time >= 1500 ) {
//    weight_ls_1 = line_sensors.weight_ls_left;
    weight_ls_1 = line_sensors.weight_ls_right;
    LeftPWM = BiasPWM + (weight_ls_1 * MaxTurnPWM);
    RightPWM = BiasPWM - (weight_ls_1 * MaxTurnPWM);
    motors.setMotorPower( LeftPWM, RightPWM ); 
  } else {
    motors.setMotorPower( 0, 0 );
  }

//  Serial.print( elapsed_time );
//  Serial.print( ",");
//  Serial.print( weight_ls_1 );
//  Serial.print( ",");
//  Serial.print( weight_ls_2 ); // obviously just inversion of other weight
//  Serial.print( "\n" );

  //  elapsed_time = SeriesDigitalReadLineSensors(LS_LEFT_PIN);
  //  Serial.print( elapsed_time );
  //  Serial.print( "," );
  //
  //  elapsed_time = SeriesDigitalReadLineSensors(LS_MIDDLE_PIN);
  //  Serial.print( elapsed_time );
  //  Serial.print( "," );
  //
  //  elapsed_time = SeriesDigitalReadLineSensors(LS_RIGHT_PIN);
  //  Serial.println( elapsed_time );


} // End of loop()
