#include "myClocks.h"  // Required for the LCD
#include "myGpio.h"    // Required for the LCD
#include "myLcd.h"     // Required for the LCD
#include <driverlib.h> // Required for the LCD
#include <msp430.h>

// TODO:
// tim0 A0: left motor
// tim0 a1: right motor

// tim1 a0: ultrasonic * 3

/* ========================= Macros ========================= */

#define ENABLE_PINS 0xFFFE // Needed to enable I/O
#define SELA_MASK 0x0300   // taken from Piazza

#define PWM_FREQUENCY 5000   // 200Hz (1MHz / 5000)
#define RAMP_INTERVAL 0x8000 // 1Hz ramping

// right motor
#define PWMA BIT4 // PWMA at P9.4
#define AIN2 BIT5 // AIN2 at P2.5
#define AIN1 BIT7 // AIN1 at P4.7

// left motor
#define PWMB BIT0 // PWMB at P9.0
#define BIN2 BIT1 // BIN2 at P4.1
#define BIN1 BIT6 // BIN1 at P9.6

// TODO: check ADC ports for echo out

// Front ultrasonic sensor
#define FRONT_TRIG_PIN BIT2 // P9.2
#define FRONT_ECHO_PIN BIT3 // P4.3

// left ultrasonic sensor
#define LEFT_TRIG_PIN BIT3 // P3.3
#define LEFT_ECHO_PIN BIT6 // P3.6

// right ultrasonic sensor
#define RIGHT_TRIG_PIN BIT1 // P2.1
#define RIGHT_ECHO_PIN BIT3 // P2.3

#define LEFT_MOTOR_FORWARD                                                     \
  do {                                                                         \
    P9OUT &= ~BIN1;                                                             \
    P4OUT |= BIN2;                                                            \
  } while (0)

#define LEFT_MOTOR_REVERSE\
  do {                                                                         \
    P9OUT |= BIN1;                                                             \
    P4OUT &= ~BIN2;                                                            \
  } while (0)

#define LEFT_MOTOR_STOP                                                        \
  do {                                                                         \
    P4OUT &= ~BIN2;                                                            \
    P9OUT &= ~BIN1;                                                            \
    P9OUT &= ~PWMB;                                                            \
  } while (0)

#define RIGHT_MOTOR_FORWARD                                                    \
  do {                                                                         \
    P4OUT |= AIN1;                                                             \
    P2OUT &= ~AIN2;                                                            \
  } while (0)

#define RIGHT_MOTOR_REVERSE\
  do {                                                                         \
    P4OUT &= ~AIN1;                                                             \
    P2OUT |= AIN2;                                                            \
  } while (0)

#define RIGHT_MOTOR_STOP                                                       \
  do {                                                                         \
    P2OUT &= ~AIN2;                                                            \
    P4OUT &= ~AIN1;                                                            \
    P9OUT &= ~PWMA;                                                            \
  } while (0)

#define START_TIMERS()                                                         \
  do {                                                                         \
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;                                   \
    TA1CTL = TASSEL__ACLK | MC__UP | TACLR;                                    \
  } while (0)

#define STOP_TIMERS()                                                          \
  do {                                                                         \
    TA0CCTL0 &= ~CCIE;                                                         \
    TA1CCTL0 &= ~CCIE;                                                         \
    TA0CTL = TASSEL__SMCLK | MC__STOP | TACLR;                                 \
    TA1CTL = TASSEL__ACLK | MC__STOP | TACLR;                                  \
  } while (0)

/* ========================= Global Vars ========================= */

volatile int frontDistance = 0;
volatile int leftDistance = -1;
volatile int rightDistance = -1;

volatile unsigned int timerCount = 0;
volatile int waitingFall = 0;

typedef enum { STOP = 0, FORWARD = 1, LEFT, RIGHT } Direction;

typedef struct motor {
  int speed;
  bool isSpinning;
} motor_t;

motor_t leftMotor = {0, false, STOP};
motor_t rightMotor = {0, false, STOP};

/* ========================= Helper Functions ========================= */

void SwitchToLFXT(void);
void SMCLK_SetTo1MHz(void);
void ultrasonicInit(void);
void motorInit(void);
void motorSetDirection(Direction direction);
void startLeftMotor(int speed);
void startRightMotor(int speed);
void stopLeftMotor(void);
void stopRightMotor(void);

// count 10us for pulse triggering (tim0)
static inline void triggerPulse() {
  P9OUT |= FRONT_TRIG_PIN; // start pulse
  __delay_cycles(10);
  P9OUT &= ~FRONT_TRIG_PIN; // end pulse
}

static inline void set65msTick() {
  TA1CCR0 = TA1R + 65000; // 65ms interval
  TA1CCTL0 = CCIE;        // enable CCR0 interrupt
}

main() {
  WDTCTL = WDTPW | WDTHOLD; // Stop WDT
  PM5CTL0 = ENABLE_PINS;    // Enable inputs and outputs

  // // Setting up LCD for debugging
  // initGPIO();
  // initClocks(); // Initialize clocks for LCD
  // myLCD_init(); // Prepares LCD to receive commands

  // FIXME: why adding lcd init blocks the motor from spinning?
  SMCLK_SetTo1MHz();
  SwitchToLFXT();

  motorInit();
  ultrasonicInit();

  __enable_interrupt(); // Activate interrupts

  // motorSetDirection(FORWARD);
  // startLeftMotor(50);
  // startRightMotor(50);

  while (1) {
    // myLCD_displayNumber(frontDistance);
    if (frontDistance > 0 && frontDistance < 20) {
      stopLeftMotor();
      stopRightMotor();
      __delay_cycles(10000);
    } else if (frontDistance >= 10) {
      if (!leftMotor.isSpinning) {
        startLeftMotor(50);
        startRightMotor(50);
      }
    }
  }
}

// TODO: change pwm duty cycle based on the distance detected
void ultrasonicInit(void) {
  // front
  P9DIR |= FRONT_TRIG_PIN; // trigger pin as output
  P9OUT &= ~FRONT_TRIG_PIN;

  P4DIR &= ~FRONT_ECHO_PIN; // echo pin as output to ADC
  P4IES &= 0x00; // detect LO --> HI transition
  P4IE |= FRONT_ECHO_PIN;   // enable interrupt
  P4IFG = 0x00;

  // // left
  // P3DIR |= LEFT_TRIG_PIN; // trigger pin as output
  // P3OUT &= ~LEFT_TRIG_PIN;

  // P3DIR &= ~LEFT_ECHO_PIN; // echo pin as output to ADC
  // P3REN &= ~LEFT_ECHO_PIN;
  // P3IES &= ~LEFT_ECHO_PIN;
  // P3IE |= LEFT_ECHO_PIN;
  // P3IFG &= ~LEFT_ECHO_PIN;

  // // right
  // P2DIR |= RIGHT_TRIG_PIN; // trigger pin as output
  // P2OUT &= ~RIGHT_TRIG_PIN;

  // P2DIR &= ~RIGHT_ECHO_PIN; // echo pin as output to ADC
  // P2REN &= ~RIGHT_ECHO_PIN;
  // P2IES &= ~RIGHT_ECHO_PIN;
  // P2IE |= RIGHT_ECHO_PIN;
  // P2IFG &= ~RIGHT_ECHO_PIN;

  // Timer A2 to measure echo width
  TA2CTL = TASSEL__SMCLK | MC__STOP | TACLR; // Use SMCLK, for UP mode, Halt timer

  // Timer A1 for triggering ultrasonic sensor every 65ms
  TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR; // SMCLK, CONTINUOUS
  
  set65msTick();
}

void motorInit(void) {
  // right motor GPIO
  P9DIR |= PWMA; // PWM output
  P9OUT &= ~PWMA;
  P2DIR |= AIN2; // H-bridge
  P4DIR |= AIN1;
  RIGHT_MOTOR_STOP;

  // left motor GPIO
  P9DIR |= PWMB; // PWM output
  P9OUT &= ~PWMB;
  P4DIR |= BIN2; // H-bridge
  P9DIR |= BIN1;
  LEFT_MOTOR_STOP;

  // Timer A0: PWM at 200Hz
  TA0CCR0 = PWM_FREQUENCY;
  TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;
  TA0CCTL1 = OUTMOD_7; // pwm mode reset/set
  TA0CCTL2 = OUTMOD_7;

  // // Timer A1: Duty cycle ramping at 1Hz (every 1sec)
  // TA1CCR0 = RAMP_INTERVAL;
  // TA1CTL = TASSEL__ACLK | MC__STOP | TACLR;
  // TA1CCTL0 = CCIE;
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
  if (speed < 0)
    speed = 0;
  if (speed > 100)
    speed = 100;

  LEFT_MOTOR_FORWARD;
  P9OUT |= PWMB;

  TA0CCR2 = (TA0CCR0 * speed) / 100; // Set PWM duty cycle

  leftMotor.speed = speed;
  leftMotor.isSpinning = true;
}

// speed at percentage (0-100%)
void setLeftMotorSpeed(int speed) {
  if (!leftMotor.isSpinning) {
    P2OUT &= ~BIN2;
    P4OUT ^= BIN1;
    P9OUT |= PWMB;
    leftMotor.isSpinning = true;
  }
}

void stopLeftMotor(void) {
  LEFT_MOTOR_STOP;
  TA0CCR2 = 0;
  leftMotor.speed = 0;
  leftMotor.isSpinning = false;
}

// speed at percentage (0-100%)
void startRightMotor(int speed) {
  if (speed < 0)
    speed = 0;
  if (speed > 100)
    speed = 100;

  RIGHT_MOTOR_FORWARD;
  P9OUT |= PWMA;

  TA0CCR1 = (TA0CCR0 * speed) / 100; // Set PWM duty cycle

  rightMotor.speed = speed;
  rightMotor.isSpinning = true;
}

void stopRightMotor(void) {
  RIGHT_MOTOR_STOP;
  TA0CCR1 = 0;
  rightMotor.speed = 0;
  rightMotor.isSpinning = false;
}

//************************************************************************
// Timer1 Interrupt Service Routine
//************************************************************************
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_ISR(void) { 
  triggerPulse();
  set65msTick(); // start another cycle
}

// FOR FRONT ULTRASONIC SENSOR
//***********************************************************************
//* Port 4 Interrupt Service Routine
//***********************************************************************
#pragma vector = PORT4_VECTOR
__interrupt void Port_4(void) {
  if (!waitingFall) {
    TA2CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR; // start timer at rising edge
    P4IES ^= FRONT_ECHO_PIN;                              // change P4 to detect HI --> LO
    waitingFall = 1;
  } else {
    timerCount = TA2R;
    TA2CTL = TASSEL__SMCLK | MC__STOP | TACLR; // stop timer and clear TimerA0 timerCount
    P4IES = 0x00;    // detect LO --> HI
    waitingFall = 0;
    frontDistance = timerCount / 58;
  }
  P4IFG &= ~FRONT_ECHO_PIN; // clear flag
}

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
      (CSCTL2 & ~SELS_3) | SELS__DCOCLK; // Route SMCLK = DCO (don't touch ACLK)
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
