class Motors_c {

  // This keyword means that all code
  // below this point is accesible from
  // outside the class.
  public:

    // These variables are "global" within the
    // class, meaning any part of the class can
    // access them.  Note, they are not global
    // from elsewhere in your Arduino program.
    // These variables will be persistent, and
    // so keep their assigned value between
    // class access operations (with the .)
    int my_int;
    float my_float;
    // https://www.pololu.com/docs/0J83/5.9
    # define L_PWM_PIN1 10
    # define L_DIR_PIN1 16
    # define R_PWM_PIN1 9
    # define R_DIR_PIN1 15
    
    # define FWD LOW
    # define REV HIGH

    // Constructor, it always needs to exist
    Motors_c() {

    }

    // Functions must be defined in the global
    // scope of the class.  E.g. you cannot
    // create a function inside a function.
    void initialise() {
      pinMode(L_PWM_PIN1, OUTPUT);
      pinMode(L_DIR_PIN1, OUTPUT);
      pinMode(R_PWM_PIN1, OUTPUT);
      pinMode(R_DIR_PIN1, OUTPUT);

      //Start a serial connection
      Serial.begin(9600);
    
      // Wait for a stable connection, report reset.
      delay(1000);
      Serial.println("***RESET***");

    }
    
    void setMotorPower(byte speed){
      // Set power for motors
      analogWrite(L_PWM_PIN1,speed);
      analogWrite(R_PWM_PIN1,speed);
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

    // You can create lots of functions, and you
    // can define them in ordinary ways to include
    // return types and arguments
    int anotherFunction( int a, float b, char c) {      

        // You can utilse a,b,c here in normal
        // ways.

        // You can also call a class function from
        // within a class, and you don't need the
        // dot, because it is local to the class.
//        myFirstFunction();

        return 0;
    }

}; // This curly-brace finishes the content of our class.
