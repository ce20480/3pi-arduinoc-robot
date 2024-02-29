//# define EMIT_PIN                   11   // Documentation says 11.
//# define LS_LEFTMOST_PIN            12   // Complete for DN1 pin
//# define LS_LEFT_PIN                18   // Complete for DN2 pin
//# define LS_MIDDLE_PIN              20   // Complete for DN3 pin
//# define LS_RIGHT_PIN               21   // Complete for DN4 pin
//# define LS_RIGHTMOST_PIN           22   // Complete for DN5 pin
//# define NUM_LS_PINS                 3   // How many pins are going to be read
//# define ANALOGUE_LS_LEFTMOST_PIN  A11   // Pin for DN1 for analogue analysis
//# define ANALOGUE_LS_LEFT_PIN       A0   // Pin for DN2 for analogue analysis
//# define ANALOGUE_LS_MIDDLE_PIN     A2   // Pin for DN3 for analogue analysis
//# define ANALOGUE_LS_RIGHT_PIN      A3   // Pin for DN4 for analogue analysis
//# define ANALOGUE_LS_RIGHTMOST_PIN  A4   // Pin for DN5 for analogue analysis
//
//
//int ls_pins[ NUM_LS_PINS ] = { LS_LEFT_PIN, LS_MIDDLE_PIN, LS_RIGHT_PIN }; // always match the entries to the max number of pins decided before, count entries from 0 in arrays
//
//float ls_reading[ NUM_LS_PINS ]; 
//
//void setup() {
//
//  // Set some initial pin modes and states
//  pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
//  pinMode( LS_LEFTMOST_PIN , INPUT );     // Set line sensor pin to input
//  pinMode( LS_LEFT_PIN , INPUT );     // Set line sensor pin to input
//  pinMode( LS_MIDDLE_PIN , INPUT );     // Set line sensor pin to input
//  pinMode( LS_RIGHT_PIN , INPUT );     // Set line sensor pin to input
//  pinMode( LS_RIGHTMOST_PIN , INPUT );     // Set line sensor pin to input
//
//  // Start Serial, wait to connect, print a debug message.
//  Serial.begin(9600);
//  delay(1500);
//  Serial.println("***RESET***");
//
//} // End of setup()
//
//unsigned long SeriesDigitalReadLineSensors( int which_pin) {
//   // Complete the steps referring to the pseudocode block
//  // Algorithm 1.
//  // The first steps have been done for you.
//  // Fix parts labelled ????
//  // Some steps are missing - add these.
//  pinMode( EMIT_PIN, OUTPUT ); // if EMIT_PIN is high then LED is used for line sensors but if low then LED is used for bump sensors( infra-red LED), 
//  // if you want to turn the EMIT_PIN off, set to input
//  digitalWrite( EMIT_PIN, HIGH );
//
//  // Step 1: charging the capacitor
//  pinMode( which_pin, OUTPUT );
//  digitalWrite( which_pin, HIGH );
//  delayMicroseconds( 10 );
//
//  pinMode( which_pin, INPUT);
//  unsigned long start_time = micros();
//  unsigned long break_time = 5000;
//
//  while( digitalRead( which_pin ) == HIGH ) {
//      // Do nothing here (waiting).
//      unsigned long end_time = micros();
//      unsigned long elapsed_time = end_time - start_time;
//      if (elapsed_time > break_time) {
//        break;
//      }
//  }
//
//  unsigned long end_time = micros();
//
//  pinMode( EMIT_PIN, INPUT );
//
//  unsigned long elapsed_time = end_time - start_time;
//
//  return elapsed_time;
//}
//
//void ParallelDigitalReadLineSensors( ) {
//
//  // we will use which to index through sensors
//  int which;
//
//  // we will count how many sensors have discharged
//  int count; 
//
//  // record start time 
//  unsigned long start_time;
//  start_time = micros();
//
//  pinMode( EMIT_PIN, OUTPUT ); // if EMIT_PIN is high then LED is used for line sensors but if low then LED is used for bump sensors( infra-red LED), 
//  // if you want to turn the EMIT_PIN off, set to input
//  digitalWrite( EMIT_PIN, HIGH );
//
//  // setup end times ( and  can ensure that they are all zero to begin with but less generalisable)
//  //  end_times_ls[0] = 0
//  //  end_times_ls[1] = 0
//  //  end_times_ls[2] = 0
//  unsigned long end_times_ls[ NUM_LS_PINS ];
//
//  for ( which = 0; which < NUM_LS_PINS; which++ ) {
//
//    // charge each of the capacitors
//    pinMode( ls_pins[ which ], OUTPUT );
//    digitalWrite( ls_pins[ which ], HIGH );
//    
//    end_times_ls[ which ] = 0;
//    
//  }
//  
//  // allow time to charge
//  delayMicroseconds(10);
//
//  // set each of the capacitors to on so they start losing charge
//  for ( which = 0; which < NUM_LS_PINS; which++ ) {
//    pinMode( ls_pins[ which ], INPUT );
//  }
//
//
//  // setup flag to identify which sensors have been read
////  boolean sensor_done[ NUM_LS_PINS ];
//  
//  // set checking parameter
//  bool done = false;
//
//  // start count at number of pins and then count down
//  count = NUM_LS_PINS;
//  
//  // loop until all 3 sensors are read( discharged)
//  while ( done == false ) {
//    for ( which = 0; which < NUM_LS_PINS; which++ ) {
//
//      // update end time
//      if ( end_times_ls[ which ] == 0 ) {
//
//        if ( digitalRead( ls_pins[ which ] ) == LOW ) {
//
//          end_times_ls[ which ] = micros();
//
//          // update count 
//          count = count - 1;
//          
//        } // check which sensors are low and update end times
//        
//      } // check if sensors has already been read
//      
//      if (count == 0)  {
//        done = true;
//      
//      } // if all sensors are read
//      
//    } // iterate through all sensors to check end times
//    
//  } // keep iterating each tick until all sensors have been checked
//
//  // save reading for each sensor
//  for ( which = 0; which < NUM_LS_PINS; which++ ) {
//    unsigned long elapsed_time;
//    elapsed_time = end_times_ls[ which ] - start_time;
//    ls_reading[ which ] = (float)elapsed_time;
//  }
//  
//}
//
//void loop() {
//
//  unsigned long elapsed_time; 
//  unsigned long end_time;
//  unsigned long start_time;
//  int my_adc_read;
//
//  my_adc_read = analogRead( ANALOGUE_LS_MIDDLE_PIN );
//  Serial.print( my_adc_read );
//  Serial.print(",");
//
//  start_time = micros();
//
//  ParallelDigitalReadLineSensors();
//
//  end_time = micros();
//  elapsed_time = end_time - start_time;
//  
////  for ( int i = 0; i < NUM_LS_PINS; i++ ){
////    Serial.print( ls_reading[ i ] );
////    Serial.print(",");
////  }
//  
//  Serial.print( elapsed_time );
//  Serial.print( "\n" );
//  
////  elapsed_time = SeriesDigitalReadLineSensors(LS_LEFT_PIN);
////  Serial.print( elapsed_time );
////  Serial.print( "," );
////
////  elapsed_time = SeriesDigitalReadLineSensors(LS_MIDDLE_PIN);
////  Serial.print( elapsed_time );
////  Serial.print( "," );
////
////  elapsed_time = SeriesDigitalReadLineSensors(LS_RIGHT_PIN);
////  Serial.println( elapsed_time );
//
//
//} // End of loop()
