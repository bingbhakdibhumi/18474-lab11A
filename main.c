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
#define PWMA BIT7 // PWMA at P1.7
#define AIN2 BIT5 // AIN2 at P2.5
#define AIN1 BIT7 // AIN1 at P4.7

// left motor
#define PWMB BIT6 // PWMB at P1.6
#define BIN2 BIT1 // BIN2 at P4.1
#define BIN1 BIT6 // BIN1 at P9.6

// Front ultrasonic sensor
#define FRONT_TRIG_PIN BIT2 // P9.2
#define FRONT_ECHO_PIN BIT3 // P4.3

// left ultrasonic sensor
#define LEFT_TRIG_PIN BIT3 // P3.3
#define LEFT_ECHO_PIN BIT6 // P3.6

// right ultrasonic sensor
#define RIGHT_TRIG_PIN BIT1 // P2.1
#define RIGHT_ECHO_PIN BIT3 // P2.3

// LED indicator (rover running or not)
#define RED_LED_PIN BIT0   // P1.0
#define GREEN_LED_PIN BIT7 // P9.7

// PID variables
#define TARGET_WALL_DISTANCE 15 // Target distance from right wall (cm)
#define BASE_SPEED 40           // Base motor speed
#define MIN_SPEED 20            // Minimum motor speed
#define MAX_SPEED 100           // Maximum motor speed

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

#define RIGHT_MOTOR_START P1OUT |= PWMA;

/* ========================= Global Vars ========================= */

volatile unsigned int frontDistance = 0;
volatile unsigned int leftDistance = 0;
volatile unsigned int rightDistance = 0;

volatile int frontWaitingFall = 0;
volatile int rightWaitingFall = 0;

volatile bool isStart = false;

typedef enum { FRONT_SENSOR = 0, LEFT_SENSOR, RIGHT_SENSOR } Sensor;
typedef enum { STOP = 0, FORWARD = 1, LEFT, RIGHT } Direction;

float integralError = 0;
float lastError = 0;
/* ========================= Helper Functions ========================= */

void SwitchToLFXT(void);
void SMCLK_SetTo1MHz(void);
static void buttonInit(void);
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
  P3OUT |= LEFT_TRIG_PIN;
  __delay_cycles(10);
  P3OUT &= ~LEFT_TRIG_PIN;
}

main() {
  WDTCTL = WDTPW | WDTHOLD; // Stop WDT
  PM5CTL0 = ENABLE_PINS;    // Enable inputs and outputs

  SMCLK_SetTo1MHz();

  // myLCD_init(); // Prepares LCD to receive commands

  buttonInit();
  motorInit();
  ultrasonicInit();

  __enable_interrupt(); // Activate interrupts

  startLeftMotor(50);
  startRightMotor(50);

  // TODO: add a button to start and stop
  while (1) {
    if (isStart) {
      if (frontDistance <= 10) {
        stopBothMotors();
        __delay_cycles(1000000);
        continue;
      }

      if (frontDistance > 40) {
        setLeftMotorSpeed(50);
        setRightMotorSpeed(50);
      } else if (frontDistance > 20) {
        setLeftMotorSpeed(20);
        setRightMotorSpeed(20);
      }

      if (rightDistance > 40) {
        motorSetDirection(FORWARD);
      } else if (rightDistance > 30) {
        motorSetDirection(RIGHT);
      } else if (rightDistance > 20) {
        motorSetDirection(LEFT);
      } else {
        motorSetDirection(STOP);
      }

      __delay_cycles(50000);
    } else {
      stopBothMotors();
    }
  }

  // while (1) {
  //   // myLCD_displayNumber(rightDistance);

  //   if (frontDistance <= 10) {
  //     stopBothMotors();
  //     __delay_cycles(1000000);
  //     continue;
  //   }

  //   if (frontDistance > 20) {
  //     pidWallFollow();
  //   } else {
  //     setLeftMotorSpeed(20);
  //     setRightMotorSpeed(20);
  //   }

  //   __delay_cycles(50000);
  // }
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

  // Timer1 A1 for triggering ultrasonic sensor every 65ms
  TA1CCR0 = 100000;
  TA1CTL = TASSEL__SMCLK | MC__UP | TACLR;
  TA1CCTL0 = CCIE; // Enable interrupt

  // Timer2 for front sensor echo timing
  TA2CTL = TASSEL__SMCLK | MC__STOP | TACLR;

  // Timer3 for right sensor echo timing
  TA3CTL = TASSEL__SMCLK | MC__STOP | TACLR;
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

  TA0CCR1 = (TA0CCR0 / 100 * speed); // Set PWM duty cycle
}

// speed at percentage (0-100%)
void setLeftMotorSpeed(int speed) {
  speed = (speed > MAX_SPEED) ? MAX_SPEED : speed;
  speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;

  TA0CCR1 = (TA0CCR0 / 100 * speed);
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

  TA0CCR2 = (TA0CCR0 / 100 * speed); // Set PWM duty cycle
}

void setRightMotorSpeed(int speed) {
  speed = (speed > MAX_SPEED) ? MAX_SPEED : speed;
  speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;

  TA0CCR2 = (TA0CCR0 / 100 * speed);
}

void stopRightMotor(void) {
  RIGHT_MOTOR_STOP;
  TA0CCR2 = 0;
}

//************************************************************************
// Timer1 Interrupt Service Routine
//************************************************************************
Sensor selectSensor = FRONT_SENSOR;
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void) {
  if (selectSensor == FRONT_SENSOR) {
    triggerFrontPulse();
    selectSensor = RIGHT_SENSOR;
  } else if (selectSensor == RIGHT_SENSOR) {
    triggerRightPulse();
    selectSensor = FRONT_SENSOR;
  }
}

// TODO: eventually change to enter clockwise or counterclockwise mode
//***********************************************************************
//* Port 1 Interrupt Service Routine (start or stop), ref: lab08c
//***********************************************************************
#pragma vector = PORT1_VECTOR
__interrupt void Port1_ISR(void) {
  __delay_cycles(10000); // quick debouncing
  if (P1IFG & BIT1) {    // P1.1 pressed
    isStart = !isStart;
    P1OUT ^= BIT0; // toggle the LEDs
    P9OUT ^= BIT7;
    P1IFG &= ~BIT1;
  }
}

//***********************************************************************
//* Port 4 Interrupt Service Routine (front ultrasonic)
//***********************************************************************
#pragma vector = PORT4_VECTOR
__interrupt void Port4_ISR(void) {
  if (P4IFG & FRONT_ECHO_PIN) {
    if (!frontWaitingFall) { // Rising edge, start timing
      TA2CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
      P4IES |= FRONT_ECHO_PIN; // detect falling edge
      frontWaitingFall = 1;
    } else { // Falling edge, stop timing
      unsigned long count = TA2R;
      TA2CTL = MC__STOP | TACLR;
      P4IES &= ~FRONT_ECHO_PIN; // detect rising edge
      frontWaitingFall = 0;
      frontDistance = count / 58;
    }
    P4IFG &= ~FRONT_ECHO_PIN;
  }
}

//***********************************************************************
//* Port 2 Interrupt Service Routine (right ultrasonic)
//***********************************************************************
#pragma vector = PORT2_VECTOR
__interrupt void Port2_ISR(void) {
  if (P2IFG & RIGHT_ECHO_PIN) {
    if (!rightWaitingFall) { // Rising edge, start timing
      TA3CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
      P2IES |= RIGHT_ECHO_PIN; // detect falling edge
      rightWaitingFall = 1;
    } else { // Falling edge, stop timing
      unsigned long count = TA3R;
      TA3CTL = MC__STOP | TACLR;
      P2IES &= ~RIGHT_ECHO_PIN; // detect rising edge
      rightWaitingFall = 0;
      rightDistance = count / 58;
    }
    P2IFG &= ~RIGHT_ECHO_PIN;
  }
}

//***********************************************************************
//* Port 3 Interrupt Service Routine (left ultrasonic)
//***********************************************************************
#pragma vector = PORT3_VECTOR
__interrupt void Port3_ISR(void) {}

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

//********************************************************************************************
// Switches ACLK to Low Frequency eXTernal crystal (LFXT) at 32.768kHz
//***********************************************************************************************
void SwitchToLFXT(void) {
  CSCTL0_H = CSKEY_H; // Unlock Clock Select (CS) registers

  PJSEL0 |= BIT4 | BIT5;    // Configure PJ.4 and PJ.5 as LFXIN/LFXOUT
  PJSEL1 &= ~(BIT4 | BIT5); // Now, they cannot be used for inputs/outputs

  CSCTL4 &= ~LFXTOFF; // Turn on 32.76kHz Low Frequency eXTernal (LFXT) signal

  do // Wait for the 32.768kHz LFXT signal to stabilize
  {
    CSCTL5 &= ~LFXTOFFG; // Clear LFXT fault flag
    SFRIFG1 &= ~OFIFG;   // Clear global Oscillator Fault Interrupt FlaG (OFIFG)
  } while (SFRIFG1 & OFIFG); // Loop until stable and OFIFG goes LO

  CSCTL1 = (CSCTL1 & ~SELA_MASK) |
           SELA__LFXTCLK; // Use 32.768kHz LFXT signal as intput to ACLK

  CSCTL0_H = 0; // Lock Clock Select (CS) registers
}

void stopBothMotors(void) {
  LEFT_MOTOR_STOP;
  RIGHT_MOTOR_STOP;
  TA0CCR1 = 0;
  TA0CCR2 = 0;
  integralError = 0; // Reset PID
  lastError = 0;
}

void pidWallFollow(void) {}
