#include "myClocks.h"  // Required for the LCD
#include "myGpio.h"    // Required for the LCD
#include "myLcd.h"     // Required for the LCD
#include <driverlib.h> // Required for the LCD
#include <msp430.h>

/* ========================= Macros ========================= */

#define ENABLE_PINS 0xFFFE // Needed to enable I/O
#define SELA_MASK 0x0300   // taken from Piazza

#define PWM_PERIOD 5000 // 200Hz (1MHz / 5000)

// right motor
#define PWMA BIT7 // PWMA at P1.7 (FIXME: also for i2c)
#define AIN2 BIT5 // AIN2 at P2.5
#define AIN1 BIT7 // AIN1 at P4.7

// left motor
#define PWMB BIT6 // PWMB at P1.6 (FIXME: also for i2c)
#define BIN2 BIT1 // BIN2 at P4.1
#define BIN1 BIT6 // BIN1 at P9.6

// Front ultrasonic sensor
#define FRONT_TRIG_PIN BIT2 // P9.2
#define FRONT_ECHO_PIN BIT3 // P4.3

// left ultrasonic sensor
#define LEFT_TRIG_PIN BIT1 // P9.1
#define LEFT_ECHO_PIN BIT2 // P4.2

// right ultrasonic sensor
#define RIGHT_TRIG_PIN BIT1 // P2.1
#define RIGHT_ECHO_PIN BIT3 // P2.3

// LED indicator (rover running or not)
#define RED_LED_PIN BIT0   // P1.0
#define GREEN_LED_PIN BIT7 // P9.7

// PID variables - NOW IN TIMER COUNTS, NOT CM
#define TARGET_WALL_DISTANCE 1160
#define BASE_SPEED 40
#define MIN_SPEED 30
#define MAX_SPEED 50
#define LEFT_MOTOR_OFFSET 0
#define RIGHT_MOTOR_OFFSET 0

#define LEFT_MOTOR_FORWARD                                                     \
  do {                                                                         \
    P9OUT &= ~BIN1;                                                            \
    P4OUT |= BIN2;                                                             \
  } while (0)

#define LEFT_MOTOR_REVERSE                                                     \
  do {                                                                         \
    P9OUT |= BIN1;                                                             \
    P4OUT &= ~BIN2;                                                            \
  } while (0)

#define LEFT_MOTOR_STOP                                                        \
  do {                                                                         \
    P4OUT &= ~BIN2;                                                            \
    P9OUT &= ~BIN1;                                                            \
    P1OUT &= ~PWMB;                                                            \
  } while (0)

#define LEFT_MOTOR_START P1OUT |= PWMB;

#define RIGHT_MOTOR_FORWARD                                                    \
  do {                                                                         \
    P4OUT |= AIN1;                                                             \
    P2OUT &= ~AIN2;                                                            \
  } while (0)

#define RIGHT_MOTOR_REVERSE                                                    \
  do {                                                                         \
    P4OUT &= ~AIN1;                                                            \
    P2OUT |= AIN2;                                                             \
  } while (0)

#define RIGHT_MOTOR_STOP                                                       \
  do {                                                                         \
    P2OUT &= ~AIN2;                                                            \
    P4OUT &= ~AIN1;                                                            \
    P1OUT &= ~PWMA;                                                            \
  } while (0)

#define GREEN_LED_ON                                                           \
  do {                                                                         \
    P1OUT &= ~RED_LED_PIN;                                                     \
    P9OUT |= GREEN_LED_PIN;                                                    \
  } while (0)

#define RED_LED_ON                                                             \
  do {                                                                         \
    P1OUT |= RED_LED_PIN;                                                      \
    P9OUT &= ~GREEN_LED_PIN;                                                   \
  } while (0)

#define LEDS_OFF                                                               \
  do {                                                                         \
    P1OUT &= ~RED_LED_PIN;                                                     \
    P9OUT &= ~GREEN_LED_PIN;                                                   \
  } while (0)

#define RIGHT_MOTOR_START P1OUT |= PWMA;

/* ========================= Global Vars ========================= */

volatile unsigned long frontDistance = 0;
volatile unsigned long leftDistance = 0;
volatile unsigned long rightDistance = 0;

volatile int frontWaitingFall = 0;
volatile int rightWaitingFall = 0;
volatile int leftWaitingFall = 0;

volatile bool isStart = false;

typedef enum { FRONT_SENSOR = 0, LEFT_SENSOR, RIGHT_SENSOR } Sensor;
typedef enum { STOP = 0, FORWARD = 1, LEFT, RIGHT } Direction;

float integralError = 0;
float lastError = 0;

// Add these global variables
volatile unsigned long frontDistanceBuffer[3] = {0};
volatile unsigned long rightDistanceBuffer[3] = {0};
volatile unsigned long leftDistanceBuffer[3] = {0};

volatile unsigned char frontBufferIndex = 0;
volatile unsigned char rightBufferIndex = 0;
volatile unsigned char leftBufferIndex = 0;

// Add this helper function
unsigned long getFilteredDistance(volatile unsigned long *buffer) {
  unsigned long sum = 0;
  int i;
  for (i = 0; i < 3; i++) {
    sum += buffer[i];
  }
  return sum / 3;
}

float error = 0;
float derivative = 0;

/* ========================= Helper Functions ========================= */

void SwitchToLFXT(void);
void SMCLK_SetTo1MHz(void);
void buttonInit(void);
void timerInit(void);
void ultrasonicInit(void);
void motorInit(void);
void motorSetDirection(Direction direction);
void startLeftMotor(int speed);
void startRightMotor(int speed);
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);
void stopLeftMotor(void);
void stopRightMotor(void);
void pidWallFollow(void);
void stopBothMotors(void);

//***********************************************************************************************
// Switches SMCLK to DCO at 1MHz
//***********************************************************************************************
void SMCLK_SetTo1MHz(void) {
#define DIVS_7 0x0070
  CSCTL0_H = CSKEY_H; // unlock CS registers
  CSCTL1 = (CSCTL1 & ~(DCORSEL | DCOFSEL_7)) |
           DCOFSEL_0; // The digitally controlled oscillator (DCO)
                      // is an internal high frequency signal that
                      // can be mapped to the SMCLK for Timer A.
                      // This command ensures DCO is at 1MHz
  CSCTL2 =
      (CSCTL2 & ~SELS_3) | SELS__DCOCLK; // Route SMCLK = DCO (don’t touch ACLK)
  CSCTL3 = (CSCTL3 & ~DIVS_7) | DIVS__1; // DCO is 1MHz, so SMCLK will be 1MHz
  SFRIFG1 &= ~OFIFG;                     // Clear oscillator fault flags
  CSCTL0_H = 0;                          // lock CS registers
}

// count 10us for pulse triggering (tim0)
static inline void triggerFrontPulse() {
  P9OUT |= FRONT_TRIG_PIN; // start pulse
  __delay_cycles(10);
  P9OUT &= ~FRONT_TRIG_PIN; // end pulse
}

static inline void triggerRightPulse() {
  P2OUT |= RIGHT_TRIG_PIN;
  __delay_cycles(10);
  P2OUT &= ~RIGHT_TRIG_PIN;
}

static inline void triggerLeftPulse() {
  P9OUT |= LEFT_TRIG_PIN;
  __delay_cycles(10);
  P9OUT &= ~LEFT_TRIG_PIN;
}

/**
 * ses the ultrasonic range finder to measure the distance
 * from the sensor to a wall. Display the distance in centimeters on the LCD
 */
main() {
  WDTCTL = WDTPW | WDTHOLD;
  PM5CTL0 = ENABLE_PINS;

  buttonInit();
  motorInit();
  ultrasonicInit();
  SMCLK_SetTo1MHz();
  timerInit();

  // myLCD_init();

  __enable_interrupt();

  startLeftMotor(BASE_SPEED);
  startRightMotor(BASE_SPEED);

  while (1) {
    // myLCD_displayNumber((unsigned long)rightDistance);
    // Front sensor: reject invalid readings
    // Max valid distance ~400cm = 23200 counts
    if (frontDistance == 0 || frontDistance > 23200) {
      continue;
    }

    // Stop if front obstacle within 15cm = 870 counts
    if (frontDistance <= 1450) { // 25cm * 58
      stopBothMotors();
      __delay_cycles(500000);
      error = 0;
      derivative = 0;
      continue;
    }

    // Check if we have a valid wall reading
    if (rightDistance > 0 && rightDistance < 5800) {
      // Normal wall following with PID
      if (rightDistance < 1160) {
        GREEN_LED_ON; // Too close
      } else if ((rightDistance > 1160) && (rightDistance < 1800)) {
        RED_LED_ON; // Too far
      } else if (rightDistance > 1800) {
        LEDS_OFF; // way too far
      }
      pidWallFollow();
    }
    __delay_cycles(10000);
  }
}

//************************************************************************
// Timer1 Interrupt Service Routine
//************************************************************************
volatile int sensorCycle = 0;

#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void) {
  sensorCycle++;

  if (sensorCycle % 3 == 0) {
    triggerFrontPulse();
  } else if (sensorCycle % 3 == 1) {
    triggerLeftPulse();
  } else {
    triggerRightPulse();
  }
}

// Add this timeout value (in microseconds at 1MHz clock)
#define ECHO_TIMEOUT 30000 // 30ms timeout (max ~500cm range)

#pragma vector = PORT4_VECTOR
__interrupt void Port4_ISR(void) {
  if (P4IFG & FRONT_ECHO_PIN) {
    if (!frontWaitingFall) { // Rising edge, start timing
      TA2CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
      P4IES |= FRONT_ECHO_PIN; // detect falling edge
      frontWaitingFall = 1;
    } else { // Falling edge, stop timing
      unsigned long count = TA2R;
      TA2CTL = TASSEL__SMCLK | MC__STOP | TACLR;
      P4IES &= ~FRONT_ECHO_PIN; // detect rising edge
      frontWaitingFall = 0;

      // Validate the count before calculating distance
      if (count < ECHO_TIMEOUT) {
        frontDistanceBuffer[frontBufferIndex] = count;
        frontDistance = getFilteredDistance(frontDistanceBuffer);
        frontBufferIndex = (frontBufferIndex + 1) % 3;
      }
      // else: ignore this reading, keep previous value
    }
    P4IFG &= ~FRONT_ECHO_PIN;
  } else if (P4IFG & LEFT_ECHO_PIN) {
    if (!leftWaitingFall) { // Rising edge, start timing
      TB0CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR;
      P4IES |= LEFT_ECHO_PIN; // detect falling edge
      leftWaitingFall = 1;
    } else { // Falling edge, stop timing
      unsigned long count = TB0R;
      TB0CTL = TBSSEL__SMCLK | MC__STOP | TBCLR;
      P4IES &= ~LEFT_ECHO_PIN; // detect rising edge
      leftWaitingFall = 0;

      // Validate the count before calculating distance
      if (count < ECHO_TIMEOUT) {
        leftDistanceBuffer[leftBufferIndex] = count;
        leftDistance = getFilteredDistance(leftDistanceBuffer);
        leftBufferIndex = (leftBufferIndex + 1) % 3;
      }
      // else: ignore this reading, keep previous value
    }
    P4IFG &= ~LEFT_ECHO_PIN;
  }
}

#pragma vector = PORT2_VECTOR
__interrupt void Port2_ISR(void) {
  if (P2IFG & RIGHT_ECHO_PIN) {
    if (!rightWaitingFall) { // Rising edge, start timing
      TA3CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
      P2IES |= RIGHT_ECHO_PIN; // detect falling edge
      rightWaitingFall = 1;
    } else { // Falling edge, stop timing
      unsigned long count = TA3R;
      TA3CTL = TASSEL__SMCLK | MC__STOP | TACLR;
      P2IES &= ~RIGHT_ECHO_PIN; // detect rising edge
      rightWaitingFall = 0;

      // Validate the count before calculating distance
      if (count < ECHO_TIMEOUT) {
        rightDistanceBuffer[rightBufferIndex] = count;
        rightDistance = getFilteredDistance(rightDistanceBuffer);
        rightBufferIndex = (rightBufferIndex + 1) % 3;
      }
      // else: ignore this reading, keep previous value
    }
    P2IFG &= ~RIGHT_ECHO_PIN;
  }
}

//***********************************************************************************************
// Buttons (P1.1: counterclockwise - left turns, P1.2: clockwise - right turns)
//***********************************************************************************************//
void buttonInit(void) {
  P1DIR |= RED_LED_PIN;
  P1OUT &= ~RED_LED_PIN;
  P9DIR |= GREEN_LED_PIN;
  P9OUT &= ~GREEN_LED_PIN;

  P1DIR &= ~(BIT1 | BIT2); // inputs
  P1REN |= (BIT1 | BIT2);  // enable pull resistors
  P1OUT |= (BIT1 | BIT2);  // pull-ups
  P1IES |= (BIT1 | BIT2);  // look for HI->LO press
  P1IFG &= ~(BIT1 | BIT2); // clear flags
  P1IE |= (BIT1 | BIT2);   // enable interrupts
}

void ultrasonicInit(void) {
  // front
  // P9.2 as output to trigger pulse (start sensing)
  P9DIR |= FRONT_TRIG_PIN;
  P9OUT &= ~FRONT_TRIG_PIN;

  // P4.3 as input from the echo pulse (returned distance)
  P4DIR &= ~FRONT_ECHO_PIN;
  P4IFG &= ~FRONT_ECHO_PIN; // clear flag
  P4IES &= ~FRONT_ECHO_PIN; // rising edge
  P4IE |= FRONT_ECHO_PIN;   // enable interrupt for pin 4.3

  // right
  P2DIR |= RIGHT_TRIG_PIN; // P2.1
  P2OUT &= ~RIGHT_TRIG_PIN;

  P2DIR &= ~RIGHT_ECHO_PIN; // P2.3
  P2IFG &= ~RIGHT_ECHO_PIN;
  P2IES &= ~RIGHT_ECHO_PIN;
  P2IE |= RIGHT_ECHO_PIN;

  // left
  P9DIR |= LEFT_TRIG_PIN;
  P9OUT &= ~LEFT_TRIG_PIN;

  P4DIR &= ~LEFT_ECHO_PIN;
  P4IFG &= ~LEFT_ECHO_PIN;
  P4IES &= ~LEFT_ECHO_PIN;
  P4IE |= LEFT_ECHO_PIN;
}

void timerInit(void) {
  // TA1 for triggering ultrasonic sensor every 65ms
  TA1CCR0 = 50000;
  TA1CTL = TASSEL__SMCLK | MC__UP | TACLR;
  TA1CCTL0 = CCIE; // Enable interrupt

  // TA2 for front sensor echo timing
  TA2CTL = TASSEL__SMCLK | MC__STOP | TACLR;

  // TA3 for right sensor echo timing
  TA3CTL = TASSEL__SMCLK | MC__STOP | TACLR;

  // TB0 for left sensor echo timing
  TB0CTL = TBSSEL__SMCLK | MC__STOP | TBCLR;
}

void motorInit(void) {
  // right motor GPIO
  P1DIR |= PWMA;  // PWM output
  P1OUT &= ~PWMA; // start low

  P1SEL0 |= PWMA; // PWM mapping (TA0.2 <-> P1.7)
  P1SEL1 |= PWMA;

  P2DIR |= AIN2; // H-bridge
  P4DIR |= AIN1;
  RIGHT_MOTOR_STOP;

  // left motor GPIO
  P1DIR |= PWMB; // PWM output
  P1OUT &= ~PWMB;

  P1SEL0 |= PWMB; // PWM mapping (TA0.1 <-> P1.6)
  P1SEL1 |= PWMB;

  P4DIR |= BIN2; // H-bridge
  P9DIR |= BIN1;
  LEFT_MOTOR_STOP;

  // Timer A0:
  TA0CCR0 = PWM_PERIOD; // PWM at 200Hz
  TA0CCR1 = 0;          // duty cycle (start w/ 0 speed)
  TA0CCR2 = 0;          // duty cycle (start w/ 0 speed)
  TA0CCTL2 = OUTMOD_7;  // reset/set PWM mode (TA0.2)
  TA0CCTL1 = OUTMOD_7;  // reset/set PWM mode (TA0.1)
  TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

// controls the direction of BOTH motors combined
// this does NOT affect the speed of the motor!
void motorSetDirection(Direction direction) {
  switch (direction) {
  case FORWARD:
    LEFT_MOTOR_FORWARD;
    RIGHT_MOTOR_FORWARD;
    break;
  case STOP:
    LEFT_MOTOR_STOP;
    RIGHT_MOTOR_STOP;
    break;
  case LEFT:
    LEFT_MOTOR_REVERSE;
    RIGHT_MOTOR_FORWARD;
    break;
  case RIGHT:
    LEFT_MOTOR_FORWARD;
    RIGHT_MOTOR_REVERSE;
  default: // no changes
    break;
  }
}

/* ========================= Motor Functions ========================= */

// controls the speed of INDIVIDUAL motor
void startLeftMotor(int speed) {
  speed = (speed > MAX_SPEED) ? MAX_SPEED : speed;
  speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;

  LEFT_MOTOR_FORWARD;
  LEFT_MOTOR_START;

  TA0CCR1 = (TA0CCR0 / 100 * (speed + LEFT_MOTOR_OFFSET));
}

// speed at percentage (0-100%)
void setLeftMotorSpeed(int speed) {
  speed = (speed > MAX_SPEED) ? MAX_SPEED : speed;
  speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;

  TA0CCR1 = (TA0CCR0 / 100 * (speed + LEFT_MOTOR_OFFSET));
}

void stopLeftMotor(void) {
  LEFT_MOTOR_STOP;
  TA0CCR1 = 0;
}

// speed at percentage (0-100%)
void startRightMotor(int speed) {
  speed = (speed > MAX_SPEED) ? MAX_SPEED : speed;
  speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;

  RIGHT_MOTOR_FORWARD;
  RIGHT_MOTOR_START

  TA0CCR2 = (TA0CCR0 / 100 * (speed + RIGHT_MOTOR_OFFSET));
}

void setRightMotorSpeed(int speed) {
  speed = (speed > MAX_SPEED) ? MAX_SPEED : speed;
  speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;

  TA0CCR2 = (TA0CCR0 / 100 * (speed + RIGHT_MOTOR_OFFSET));
}

void stopRightMotor(void) {
  RIGHT_MOTOR_STOP;
  TA0CCR2 = 0;
}

void stopBothMotors(void) {
  LEFT_MOTOR_STOP;
  RIGHT_MOTOR_STOP;
  TA0CCR1 = 0;
  TA0CCR2 = 0;
  integralError = 0; // Reset PID
  lastError = 0;
}
volatile float filteredError = 0;

void pidWallFollow() {
  const float Kp = 0.0179;
  const float Kd = 0.801;

  error = (float)rightDistance - TARGET_WALL_DISTANCE;

  // simple LPF
  filteredError = 0.7f * filteredError + 0.3f * error;

  derivative = filteredError - lastError;
  lastError = filteredError;

  if (error > 640) {
    if (derivative < 400) {
      // Lost wall, gently turn left
      error = 0;
      derivative = 0;
      setLeftMotorSpeed(BASE_SPEED - 2);
      setRightMotorSpeed(BASE_SPEED);
    }
    return; // open door case, ignore sudden changes in distance reading
  }

  float output = Kp * filteredError + Kd * derivative;

  // rate limit
  if (output > 7)
    output = 7;
  if (output < -7)
    output = -7;

  int left = BASE_SPEED + (int)output;

  int right = BASE_SPEED - (int)output;

  setLeftMotorSpeed(left);
  setRightMotorSpeed(right);
}
