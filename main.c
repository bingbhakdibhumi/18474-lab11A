#include "myClocks.h"  // Required for the LCD
#include "myGpio.h"    // Required for the LCD
#include "myLcd.h"     // Required for the LCD
#include <driverlib.h> // Required for the LCD
#include <msp430.h>

/* ========================= Macros ========================= */

#define ENABLE_PINS 0xFFFE // Needed to enable I/O
#define SELA_MASK 0x0300   // taken from Piazza

#define PWM_PERIOD 5000      // 200Hz (1MHz / 5000)
#define RAMP_INTERVAL 0x8000 // 1Hz ramping

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

volatile int frontDistance = -1;
volatile int leftDistance = -1;
volatile int rightDistance = -1;

volatile unsigned int timerCount = 0;
volatile int waitingFall = 0;

typedef enum { STOP = 0, FORWARD = 1, LEFT, RIGHT } Direction;

/* ========================= Helper Functions ========================= */

void SwitchToLFXT(void);
void SMCLK_SetTo1MHz(void);
void ultrasonicInit(void);
void motorInit(void);
void motorSetDirection(Direction direction);
void startLeftMotor(int speed);
void startRightMotor(int speed);
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);
void stopLeftMotor(void);
void stopRightMotor(void);

// count 10us for pulse triggering (tim0)
static inline void triggerPulse() {
  P3OUT |= FRONT_TRIG_PIN; // start pulse
  __delay_cycles(10);
  P3OUT &= ~FRONT_TRIG_PIN; // end pulse
}

main() {
  WDTCTL = WDTPW | WDTHOLD; // Stop WDT
  PM5CTL0 = ENABLE_PINS;    // Enable inputs and outputs

  // myLCD_init(); // Prepares LCD to receive commands

  // FIXME: why adding lcd init blocks the motor from spinning?
  SMCLK_SetTo1MHz();
  SwitchToLFXT();

  motorInit();
  // ultrasonicInit();

  __enable_interrupt(); // Activate interrupts

  while (1) {
    // if (frontDistance > 0 && frontDistance < 10) {
    //   stopLeftMotor();
    //   stopRightMotor();
    // } else if (frontDistance >= 10) {
    //   if (!leftMotor.isSpinning) {
    // startLeftMotor(50);
    // startRightMotor(50);
    //   }
    // }
    // __delay_cycles(10000);

    startLeftMotor(30);
    startRightMotor(30);
    __delay_cycles(500000);

    setLeftMotorSpeed(100);
    setRightMotorSpeed(100);
    __delay_cycles(500000);

    setLeftMotorSpeed(50);
    setRightMotorSpeed(50);
    __delay_cycles(500000);

    motorSetDirection(LEFT);
    __delay_cycles(500000);

    motorSetDirection(RIGHT);
    __delay_cycles(500000);

    stopLeftMotor();
    stopRightMotor();
    __delay_cycles(500000);
  }
}

// TODO: change pwm duty cycle based on the distance detected
void ultrasonicInit(void) {
  // front
  P3DIR |= FRONT_TRIG_PIN; // trigger pin as output
  P3OUT &= ~FRONT_TRIG_PIN;

  P3DIR &= ~FRONT_ECHO_PIN; // echo pin as output to ADC
  P3REN &= ~FRONT_ECHO_PIN;
  P3IES &= ~FRONT_ECHO_PIN;
  P3IE |= FRONT_ECHO_PIN;
  P3IFG &= ~FRONT_ECHO_PIN;

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

  // timer init
  TA2CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;

  // Timer A1 for triggering ultrasonic sensor every 65ms
  TA1CCR0 = 2130; // ~65ms at 32.768kHz (65ms * 32768Hz / 1000)
  TA1CTL = TASSEL__ACLK | MC__UP | TACLR;
  TA1CCTL0 = CCIE; // Enable interrupt
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
  speed = (speed > 100) ? 100 : speed;
  speed = (speed < 0) ? 0 : speed;

  LEFT_MOTOR_FORWARD;
  LEFT_MOTOR_START;

  TA0CCR1 = (TA0CCR0 / 100 * speed); // Set PWM duty cycle
}

// speed at percentage (0-100%)
void setLeftMotorSpeed(int speed) { TA0CCR1 = (TA0CCR0 / 100 * speed); }

void stopLeftMotor(void) {
  LEFT_MOTOR_STOP;
  TA0CCR1 = 0;
}

// speed at percentage (0-100%)
void startRightMotor(int speed) {
  speed = (speed > 100) ? 100 : speed;
  speed = (speed < 0) ? 0 : speed;

  RIGHT_MOTOR_FORWARD;
  RIGHT_MOTOR_START

  TA0CCR2 = (TA0CCR0 / 100 * speed); // Set PWM duty cycle
}

void setRightMotorSpeed(int speed) { TA0CCR2 = (TA0CCR0 / 100 * speed); }

void stopRightMotor(void) {
  RIGHT_MOTOR_STOP;
  TA0CCR2 = 0;
}

//************************************************************************
// Timer1 Interrupt Service Routine
//************************************************************************
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_ISR(void) { triggerPulse(); }

//***********************************************************************
//* Port 4 Interrupt Service Routine
//***********************************************************************
#pragma vector = PORT4_VECTOR
__interrupt void Port_4(void) {
  if (!waitingFall) {
    TA0CTL =
        TASSEL__SMCLK | MC__CONTINUOUS | TACLR; // start timer at rising edge
    P4IES ^= BIT3;                              // change P4 to detect HI --> LO
    waitingFall = 1;
  } else {
    timerCount = TA0R;
    TA0CTL = TASSEL__SMCLK | MC__STOP;
    TA0CTL |= TACLR; // clear TimerA0 timerCount
    P4IES = 0x00;    // detect LO --> HI
    waitingFall = 0;
    frontDistance = timerCount / 58;
  }
  P4IFG &= ~BIT3; // clear flag
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
