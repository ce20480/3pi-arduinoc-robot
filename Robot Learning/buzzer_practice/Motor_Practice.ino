//// Replace the ? with correct pin numbers
//// https://www.pololu.com/docs/0J83/5.9
//# define L_PWM_PIN1 10
//# define L_PWM_PIN2 A10
//# define L_PWM_PIN3 28
//# define L_DIR_PIN1 16
//# define L_DIR_PIN2 MOSI
//# define R_PWM_PIN1 9
//# define R_PWM_PIN2 A9
//# define R_PWM_PIN3 27
//# define R_DIR_PIN1 15
//# define R_DIR_PIN2 SCK
//
//# define FWD LOW
//# define REV HIGH
//
//// Runs once.
//void setup() {
//
//  // Set all the motor pins as outputs.
//  // There are 4 pins in total to set.
//  pinMode(L_PWM_PIN1, OUTPUT);
//  pinMode(L_DIR_PIN1, OUTPUT);
//  pinMode(R_PWM_PIN1, OUTPUT);
//  pinMode(R_DIR_PIN1, OUTPUT);
//
//  // Set initial direction (HIGH/LOW)
//  // for the direction pins.
//  // ...
//
//  // Set initial power values for the PWM
//  // Pins.
//  // ...
//
//
//  // Start serial, send debug text.
//  Serial.begin(9600);
//  delay(1000);
//  Serial.println("***RESET***");
//
//}
//
//// Repeats.
//void loop() {
//
//  // Add code to set the direction of rotation
//  // for the left and right motor here.
//
//
//  // Use analogWrite() to set the power to the
//  // motors.
//  // 20 is a low level of activation to test
//  // the motor operation.
//  analogWrite( L_PWM_PIN, 20 );
//  analogWrite( R_PWM_PIN, 20 );
//
//  // An empty loop can block further uploads.
//  // A small delay to prevent this for now.
//  delay(5);
//}
