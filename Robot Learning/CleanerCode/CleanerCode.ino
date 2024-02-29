# include "motor.h"
# include "linesensor.h"
# include "statechecker.h"

Motors_c Motors_class;
LineSensor_c Linesensor_class;
StateChecker_c Statechecker_class;

//typedef enum {
//
//  OFF_LINE = 0,
//  ON_LINE = 1,
//
//} STATE;

int state;

void setup() {
  
  // put your setup code here, to run once:
  Linesensor_class.initialise_digital();
  Motors_class.initialise();
  
}

void loop() {
  
  // put your main code here, to run repeatedly:

  state = Statechecker_class.UpdateStateTest( state );
//  unsigned long elapsed_time = Linesensor_class.ReadRightmostSensor();
//  Serial.print( state );
//  Serial.print(",");
//  Serial.println(elapsed_time);
  Statechecker_class.UpdateActionTest( state );

//  state = Statechecker_class.UpdateState( state );
//  Statechecker_class.UpdateAction( state );

}
