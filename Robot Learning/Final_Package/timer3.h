#ifndef _TIMER3_H
#define _TIMER3_H

volatile float left_angular_speed;
volatile float right_angular_speed;
volatile float avg_angular_left_speed;
volatile float avg_angular_right_speed;

volatile long count_angular_left_last; // for the difference in encoder counts
volatile long count_angular_right_last;
volatile long diff_right_count;
volatile long diff_left_count;

const float elapsed_time = 1 * 1e1; // measured in ms ( milliseconds )

volatile unsigned long isr_start_time_t3 = 0;
volatile unsigned long isr_end_time_t3 = 0;
volatile unsigned long isr_execution_time_t3 = 0;

// Routine to setupt timer3 to run
void setupTimer3() {

  //  isr_start_time_t3 = micros(); // 130 microseconds for the right

  // disable global interrupts
  cli();

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual.
  TCCR3B = TCCR3B | (1 << CS32);


  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  //  OCR3A = 31250;
  // if we want 4hz = 1/4 seconds then we need OCR3A 62500 / 4
  // if we want 1ms = 10e-3s so 62500/1000 = 62.5 or 1microseconds is 62500/1,000,000 = 0.0625
  OCR3A = 626; // 100hz
  //  OCR3A = 1249; // 50hz

  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();

  //  isr_end_time_t3 = micros();
  //  isr_execution_time_t3 = isr_end_time_t3 - isr_start_time_t3;

}

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {

  isr_start_time_t3 = micros(); // 130 microseconds for the right 0.13 milliseconds


  // Find difference in encoder counts
  diff_right_count = count_e_right - count_angular_right_last;
  diff_left_count = count_e_left - count_angular_left_last;

  // Update new speeds
  right_angular_speed = (float)diff_right_count;
  right_angular_speed /= elapsed_time;
  left_angular_speed = (float)diff_left_count;
  left_angular_speed /= elapsed_time;

  // avg to get even more accurate
  avg_angular_right_speed = ( avg_angular_right_speed * 0.4 ) + ( right_angular_speed * 0.6 );
  avg_angular_left_speed = ( avg_angular_left_speed * 0.4 ) + ( left_angular_speed * 0.6 );

  // Update old count values
  count_angular_right_last = count_e_right;
  count_angular_left_last = count_e_left;

  //  Serial.print(diff_left_count);
  //  Serial.print(", ");
  //  Serial.println(diff_right_count);

  isr_end_time_t3 = micros();
  isr_execution_time_t3 = isr_end_time_t3 - isr_start_time_t3;

}

#endif
