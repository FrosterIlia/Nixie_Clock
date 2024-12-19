#include "Arduino.h"
#include <GyverPWM.h>
#include "PID.h"
#include "Timer.h"

#define _A0 2
#define _A1 8
#define _A2 7
#define _A3 3

#define OPT1_PIN 13
#define OPT2_PIN 10

#define DIGITS_NUMBER 4


Pid pid(6, 30, 0, 10);
const int NUM_READ = 30;

void setup() {
  Serial.begin(9600);
  pid.set_limits(0, 255);
  pinMode(9, OUTPUT);
  pinMode(A0, INPUT);

  pinMode(_A0, OUTPUT);
  pinMode(_A1, OUTPUT);
  pinMode(_A2, OUTPUT);
  pinMode(_A3, OUTPUT);

  pinMode(OPT1_PIN, OUTPUT);
  pinMode(OPT2_PIN, OUTPUT);

  //interrupts();
  setupTimer2();
}

uint8_t duty = 0;
uint16_t freq = 14000;

uint16_t des_voltage = 10;
uint16_t current_voltage = 0;

Timer main_timer(100);
Timer pid_timer(10);

uint8_t opt_pins[] = {OPT1_PIN, OPT2_PIN, 12, 11};
uint8_t A_pins[] = {_A0, _A1, _A2, _A3};

uint8_t digits[DIGITS_NUMBER];

int value;

void loop() {



  if (pid_timer.isReady()) {
    current_voltage = midArifm();
    pid.input = current_voltage;
    pid.setpoint = des_voltage;
    pid.compute();
  }

  if (Serial.available() > 1) {
    char symbol;
    Serial.readBytes(&symbol, 1);

    switch (symbol) {
      // case 'd':
      //   duty = Serial.parseInt();
      //   break;

      case 'f':
        freq = Serial.parseInt();
        break;

      case 'p':
        pid.kp = Serial.parseFloat();
        break;

      case 'i':
        pid.ki = Serial.parseFloat();
        break;

      case 's':
        des_voltage = Serial.parseInt();
        break;

      case 'd':
        pid.kd = Serial.parseFloat();
        break;



      case 'n': //number
        value = Serial.parseInt();
        
        // Parsing the digits
        uint8_t number_length = count_digits(value);
        uint16_t divisor = myPow(10, number_length - 1);
        for (uint8_t i = 0; i < number_length; i++) {

          digits[i] = value / divisor;

          value %= divisor;
          divisor /= 10;
        }

        break;
    }
  }
  PWM_set(9, constrain(pid.get_output(), 0, 249));

  if (main_timer.isReady()) {
    Serial.println(map(analogRead(A0), 0, 1023, 2, 233));
    Serial.print(" ");
    Serial.print(pid.get_output());
  Serial.print(" ");
  Serial.print(des_voltage);
  Serial.print(" ");
  Serial.println(current_voltage);
    PWM_frequency(9, freq, CORRECT_PWM);
  }
}

int midArifm() {
  long sum = 0;                       
  for (int i = 0; i < NUM_READ; i++)  
    sum += map(analogRead(A0), 0, 1023, 2, 233);
  return ((float)sum / NUM_READ);
}


void setupTimer2(){

  //TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22); // 1024 prescaler
  // minimum frequency = 60Hz

  TCCR2B = (TCCR2B & B11111000) | 3;    //prescaler 64
  TCCR2A |= (1 << WGM21);   // CTC Mode
  TIMSK2 |= (1 << OCIE2A);  // Turn on the interrupt
}

volatile uint8_t opt_counter = 0;

ISR(TIMER2_COMPA_vect){ //TODO turn off the optocouple right before turning on another one
  setPin(opt_pins[opt_counter], 1);
  setPin(_A0, digits[opt_counter] & 0x01);
  setPin(_A1, digits[opt_counter] & 0x02);
  setPin(_A2, digits[opt_counter] & 0x04);
  setPin(_A3, digits[opt_counter] & 0x08);
  setPin(opt_pins[opt_counter], 0);
  if (++opt_counter >= DIGITS_NUMBER) opt_counter = 0;
}



uint8_t count_digits(uint32_t number) {
  if (number == 0) {
    return 1;  // Special case for 0
  }

  uint8_t digits = 0;
  while (number > 0) {
    number /= 10;  // Remove the last digit
    digits++;      // Increment digit count
  }

  return digits;
}

uint16_t myPow(uint16_t number, uint16_t power){
  
  uint16_t result = number;
  for (uint8_t i = 0; i < power - 1; i++){
    
    result *= number;
  }

  return result;
}


void setPin(uint8_t pin, uint8_t x) {
  switch (pin) { // откл pwm
    case 3:  // 2B
      bitClear(TCCR2A, COM2B1);
      break;
    case 5: // 0B
      bitClear(TCCR0A, COM0B1);
      break;
    case 6: // 0A
      bitClear(TCCR0A, COM0A1);
      break;
    case 9: // 1A
      bitClear(TCCR1A, COM1A1);
      break;
    case 10: // 1B
      bitClear(TCCR1A, COM1B1);
      break;
    case 11: // 2A
      bitClear(TCCR2A, COM2A1);
      break;
  }

  if (pin < 8) bitWrite(PORTD, pin, x);
  else if (pin < 14) bitWrite(PORTB, (pin - 8), x);
  else if (pin < 20) bitWrite(PORTC, (pin - 14), x);
  else return;
}