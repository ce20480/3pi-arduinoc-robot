// Replace the ? with correct pin numbers
// https://www.pololu.com/docs/0J83/5.9
# define L_PWM_PIN1 10
# define L_PWM_PIN2 A10
# define L_PWM_PIN3 28
# define L_DIR_PIN1 16
# define L_DIR_PIN2 MOSI
# define R_PWM_PIN1 9
# define R_PWM_PIN2 A9
# define R_PWM_PIN3 27
# define R_DIR_PIN1 15
# define R_DIR_PIN2 SCK

# define FWD LOW
# define REV HIGH

// Runs once.
void setup() {

  // Set all the motor pins as outputs.
  // There are 4 pins in total to set.
  pinMode(L_PWM_PIN1, OUTPUT);
  pinMode(L_DIR_PIN1, OUTPUT);
  pinMode(R_PWM_PIN1, OUTPUT);
  pinMode(R_DIR_PIN1, OUTPUT);
  pinMode(L_PWM_PIN2, OUTPUT);
  pinMode(L_DIR_PIN2, OUTPUT);
  pinMode(R_PWM_PIN2, OUTPUT);
  pinMode(R_DIR_PIN2, OUTPUT);
  pinMode(L_PWM_PIN3, OUTPUT);
  pinMode(R_PWM_PIN3, OUTPUT);

  // Set initial direction (HIGH/LOW)
  // for the direction pins.
  digitalWrite(L_DIR_PIN2, FWD);
  digitalWrite(R_DIR_PIN2, FWD);

  // Set initial power values for the PWM
  // Pins.
//  analogWrite(L_PWM_PIN1,100);
//  analogWrite(R_PWM_PIN1,100);


  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

}

// turn right 
void TurnRight(byte speed){
  // Set initial direction (HIGH/LOW)
  // for the direction pins.
  digitalWrite(L_DIR_PIN1, FWD);
  digitalWrite(R_DIR_PIN1, REV);
  analogWrite(L_PWM_PIN1,speed);
  analogWrite(R_PWM_PIN1,speed);
}

// turn left 
void TurnLeft(byte speed){
  // Set initial direction (HIGH/LOW)
  // for the direction pins.
  digitalWrite(L_DIR_PIN1, REV);
  digitalWrite(R_DIR_PIN1, FWD);
  analogWrite(L_PWM_PIN1,speed);
  analogWrite(R_PWM_PIN1,speed);
}

// move forward
void MoveForward(byte speed){
  // Set initial direction (HIGH/LOW)
  // for the direction pins.
  digitalWrite(L_DIR_PIN1, FWD);
  digitalWrite(R_DIR_PIN1, FWD);
  analogWrite(L_PWM_PIN1,speed);
  analogWrite(R_PWM_PIN1,speed);
}

// turn right 
void MoveReverse(byte speed){
  // Set initial direction (HIGH/LOW)
  // for the direction pins.
  digitalWrite(L_DIR_PIN1, REV);
  digitalWrite(R_DIR_PIN1, REV);
  analogWrite(L_PWM_PIN1,speed);
  analogWrite(R_PWM_PIN1,speed);
}

// Repeats.
void loop() {

  // Add code to set the direction of rotation
  // for the left and right motor here.


  // Use analogWrite() to set the power to the
  // motors.
  // 20 is a low level of activation to test
  // the motor operation.
//  int x = 0;
  int i = 0;
  do {
    delay(20);
    analogWrite( L_PWM_PIN3, 50 );
    analogWrite( R_PWM_PIN3, 50 );
    i++;
  } while (i < 10);

  // An empty loop can block further uploads.
  // A small delay to prevent this for now.
  delay(5);
}
