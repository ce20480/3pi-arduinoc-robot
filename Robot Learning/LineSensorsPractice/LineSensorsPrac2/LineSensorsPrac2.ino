# define EMIT_PIN    11    // Documentation says 11.
# define LS_LEFT_PIN 12   // Complete for DN1 pin

# define MAX_SAMPLES 10
float results[ MAX_SAMPLES ]; // An array of MAX_SAMPLES length

void setup() {

  // Set some initial pin modes and states
  pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
  pinMode( LS_LEFT_PIN, INPUT );     // Set line sensor pin to input

  // Start Serial, wait to connect, print a debug message.
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");

} // End of setup()


void loop() {

  int i = 0;
  Serial.println(i);
  while( i < 10 ) {
    // Complete the steps referring to the pseudocode block
    // Algorithm 1.
    // The first steps have been done for you.
    // Fix parts labelled ????
    // Some steps are missing - add these.
    pinMode( EMIT_PIN, OUTPUT );
    digitalWrite( EMIT_PIN, HIGH );
  
  
    pinMode( LS_LEFT_PIN, OUTPUT );
    digitalWrite( LS_LEFT_PIN, HIGH );
    delayMicroseconds( 10 );
  
  
    unsigned long start_time = micros();
  
    while( digitalRead( LS_LEFT_PIN ) == HIGH ) {
        // Do nothing here (waiting).
        Serial.println(i);
    }
  
    unsigned long end_time = micros();
  
    pinMode( EMIT_PIN, INPUT );
  
    unsigned long elapsed_time = end_time - start_time;
  
    // This line is complete, but it needs to be
    // within a for loop construct.  Complete this.
    results[i] = (float)elapsed_time;
    
    delay(200);
  
  
    // The section below has been completed for you.
    // Use this section to build a similar construct
    // around the code above, which was Algorithm 1.
  
    // The robot will be "stuck" here forever, because
    // the condition true will never change.
    // Therefore, the robot will Serial Print the
    // results back to your computer every 1000ms.
    i++;
    delay(1000);
  }

  
  Serial.println("Results: ");
  for( int i = 0; i < MAX_SAMPLES; i++ ) {
    Serial.println( results[i] );
  }
  
} // End of loop()
