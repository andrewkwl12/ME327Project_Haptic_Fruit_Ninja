 /*--- USER INPUTS ---*/
#define PRINT_RATE 50 // Period of printing to serial
#define DT 20 // Simulation time
#define VIBE_PERIOD 4 // Time between changing pin dir
#define VIBE_TIME 100
#define MAX_FRUITS 1 // Amount of fruits in play
#define FRUIT_DIAM 2.0 // Fruit diameter in meters (Yes, I know these are giant fruits. It's okay.)
#define CANVAS_WIDTH 10 // Width of virtual environment in simulation in meters
#define BAUD_RATE 115200 // Serial baud rate

/*--- IMPORTS ---*/
#include <math.h>
#include <arduino-timer.h>
#include <AS5048A.h>
#include "Fruit.h"

/*--- VALUE DEFINITIONS ---*/
#define PWM_PIN1 5
#define DIR_PIN1 8
#define PWM_PIN2 6
#define DIR_PIN2 7
#define SENSOR_POS_PIN A2
#define FSR_PIN A3
#define PRESCALE_FACTOR 64

/*--- CREATE CLASS INSTANCES ---*/
auto printTimer = timer_create_default();
auto simulationTimer = timer_create_default();
Fruit fruitVect[MAX_FRUITS];
AS5048A angleSensor(10);

/*--- FUNCTION PROTOTYPES ---*/
void setPwmFrequency(int, int);
bool printStuff(void *);
bool simulate(void *);
int modMillis(void);
void setSimTime(int);
void initializeFruits(int,float);
int numIntersected(int,int);
float getPositionMR(void);
float getPositionHallEffect(void);
float pen_x(float,float);
float pen_y(float,float);
float find_x_Cursor(float);
float find_y_Cursor(float);
void vibrateMotors(int);

/*--- MODULE VARIABLES ---*/
static bool simulateFlag = false; // This is set to true in the state machine
static float xCursor = CANVAS_WIDTH/2.0; // The x position of the cursor in the game canvas
static float yCursor = CANVAS_WIDTH/2.0; // The y position of the cursor in the game canvas
static float xCursorPrev = CANVAS_WIDTH/2.0; // For rejecting NaN
static float yCursorPrev = CANVAS_WIDTH/2.0; // For rejecting NaN
static int numIntersectedFruits;
static int lastNumIntersectedFruits;
static volatile int vibeDirState = 1;
static long vibeTimestamp = 0;
static volatile int vibeFlag = 0;

// Printing vars
bool testMode = false;
float printVar1 = 0;
float printVar2 = 0;

// Hall Effect sensor vars
static long updatedPos = 0;
static int rawPos = 0;
static int lastRawPos = 0;
static int lastLastRawPos = 0;
static int flipNumber = 0;
static int tempOffset = 0;
static int rawDiff = 0;
static int lastRawDiff = 0;
static int rawOffset = 0;
static int lastRawOffset = 0;
static const int flipThresh = 10000; // previously 16000
static boolean flipped = false;
static double OFFSET = 16000;
static double OFFSET_NEG = 15;
static double ts = 0;
static double xh = 0;

// MR sensor vars
static long updatedPos_MR = 0;
static int rawPos_MR = 0;
static int lastRawPos_MR = 0;
static int lastLastRawPos_MR = 0;
static int flipNumber_MR = 0;
static int tempOffset_MR = 0;
static int rawDiff_MR = 0;
static int lastRawDiff_MR = 0;
static int rawOffset_MR = 0;
static int lastRawOffset_MR = 0;
static const int flipThresh_MR = 700;
static boolean flipped_MR = false;
static double OFFSET_MR = 980;
static double OFFSET_NEG_MR = 15;
static double ts_MR = 0;
static double xh_MR = 0;

/*--- CONSTANTS ---*/
// Pantograph dimensions
const float a1 = 5.0;
const float a2 = 6.0;
const float a3 = 6.0;
const float a4 = 5.0;
const float a5 = 3.25;

/*--- STATE MACHINE VARIABLES ---*/
typedef enum {
  FRUIT_NINJA
} RobotStates_t;

RobotStates_t RobotState;

typedef enum {
  VIBRATE,STANDBY
} VibrateState_t;

VibrateState_t VibrateState;

void setup() {
  RobotState = FRUIT_NINJA;
  VibrateState = STANDBY;
  
  Serial.begin(BAUD_RATE);
  while(!Serial);

  // Set PWM frequency
//  setPwmFrequency(PWM_PIN1,1);
//  setPwmFrequency(PWM_PIN2,1);

  // Initialize timers
  printTimer.every(PRINT_RATE, printStuff);
  simulationTimer.every(DT, simulate);

  // Initialize fruits (set sim time and fruit diameter)
  initializeFruits(DT,FRUIT_DIAM);

  // Initialize Hall Effect sensor
  angleSensor.init();
  angleSensor.getRawRotation();
  lastLastRawPos = angleSensor.getRawRotation(); // position from Hall Effect sensor
  lastRawPos = angleSensor.getRawRotation(); // position from Hall Effect sensor
  flipNumber = 0;

  // Initialize MR sensor
  pinMode(SENSOR_POS_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);
  lastLastRawPos_MR = analogRead(SENSOR_POS_PIN);
  lastRawPos_MR = analogRead(SENSOR_POS_PIN);
  flipNumber_MR = 0;

  // Define motor output pins
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);

  pinMode(PWM_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);


  // Initialize motor values
  analogWrite(PWM_PIN1, 0);
  digitalWrite(DIR_PIN1, LOW);

  analogWrite(PWM_PIN2, 0);
  digitalWrite(DIR_PIN2, LOW);

  // Initialize interrupt timer
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void loop() {
  switch(RobotState){
    case FRUIT_NINJA:{
      // Tick the print timer
      printTimer.tick();
      
      // Tick the simulation timer
      simulationTimer.tick();
      
      simulateFlag = true;
      
      // Number of fruits intersecting with cursor
      numIntersectedFruits = numIntersected(xCursor,yCursor);
      
      // Calculate motor positions
      float ts_MR = getPositionMR(); // theta_1 
      float ts_HE = getPositionHallEffect(); // theta_5
      ts_MR = ts_MR - 0.2;
      ts_HE = ts_HE + 0.12;
      float theta_1 = ts_MR + PI/4.0;
      float theta_5 = PI - (ts_HE + PI/4.0);

      // Calculate end effector position in game canvas
      xCursor = find_x_Cursor(pen_x(theta_1, theta_5));
      yCursor = find_y_Cursor(pen_y(theta_1, theta_5));

      // Check for NaN values
      if(isnan(xCursor)){
        xCursor = xCursorPrev;
      }else{
        xCursorPrev = xCursor;
      }

      // Check for NaN values
      if(isnan(yCursor)){
        yCursor = yCursorPrev;
      }else{
        yCursorPrev = yCursor;
      }

      printVar1 = theta_1;
      printVar2 = theta_5;
    }
    break;
  }

  switch(VibrateState){
    case STANDBY:{
      if(numIntersectedFruits > 0 && lastNumIntersectedFruits == 0){
        vibeFlag = 1;
        vibeTimestamp = millis();
        VibrateState = VIBRATE;
      }
    }
    break;
    
    case VIBRATE:{
      if(millis()-vibeTimestamp > VIBE_TIME){
        vibeFlag = 0;
        VibrateState = STANDBY;
      }
    }
    break;
  }
  lastNumIntersectedFruits = numIntersectedFruits;
}

/* --- ISR --- */
// Interrupt is called once a millisecond 
SIGNAL(TIMER0_COMPA_vect) {
  static int vibeCounter = 0;
  if(vibeCounter >= VIBE_PERIOD){
    vibeDirState = !vibeDirState;
    vibeCounter = 0;
  }
  vibeCounter = vibeCounter + 1;
  vibrateMotors(vibeFlag);
}

// Sets pwm frequency of output pin
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

// Prints to serial monitor. Called with a timer.
bool printStuff(void *){
  if(!testMode){
    for(int i=0; i<MAX_FRUITS; i++){
      Serial.print(fruitVect[i].getX());
      Serial.print(",");
    }
    for(int i=0; i<(MAX_FRUITS-1); i++){
      Serial.print(fruitVect[i].getY());
      Serial.print(",");
    }
    Serial.print(fruitVect[MAX_FRUITS-1].getY());
    Serial.print(",");
    Serial.print(xCursor);
    Serial.print(",");
    Serial.print(yCursor);
    Serial.print(",");
    Serial.println(numIntersectedFruits);
  }else{
    Serial.println(vibeDirState);
//    Serial.print(printVar1);
//    Serial.print(",");
//    Serial.println(printVar2);
  }
  return true;
}

// Launches fruits and simulates their trajectories. Called with a timer.
bool simulate(void *){
  const int MAX_INIT_Y_VEL = 10;
  const int MIN_INIT_Y_VEL = 4;
  const int MAX_INIT_X_VEL = 5;
  const float SCALE = 100;
  
  if(simulateFlag){
    for(int i = 0; i<MAX_FRUITS; i++){
      if(!fruitVect[i].isInPlay()){ // Launch fruits that are not in play
        float randX = random(CANVAS_WIDTH*SCALE)/SCALE;
        float randXDot = random(MAX_INIT_X_VEL*SCALE)/SCALE;
        if(randX > CANVAS_WIDTH/2){ // Always launch towards center of canvas
          randXDot = randXDot*-1.0;
        }
        float randYDot = random(MIN_INIT_Y_VEL*SCALE,MAX_INIT_Y_VEL*SCALE)/SCALE;
        fruitVect[i].setPos(randX,0);
        fruitVect[i].setVel(randXDot,randYDot);
      }
      fruitVect[i].updatePos(); // Timestep simulatiom
    }
  }
  return true;
}

// Returns correct time in milliseconds
int modMillis(void){
  return millis()/PRESCALE_FACTOR;
}

// Sets fruit simulation time and diameter
void initializeFruits(int simTime, float fruitDiam){
  for(int i = 0; i<MAX_FRUITS; i++){
    fruitVect[i].setDt(simTime);
    fruitVect[i].setDiam(fruitDiam);
  }
}

// Checks how many fruits are intersected with a given coordinate on the canvas
int numIntersected(int xCursor, int yCursor){
  int numIntersect = 0;
  for(int i = 0; i<MAX_FRUITS; i++){
    numIntersect = numIntersect + fruitVect[i].isIntersected(xCursor,yCursor);
  }
  return numIntersect;
}

// Updates position of MR sensor
float getPositionMR(void){
  rawPos_MR = analogRead(SENSOR_POS_PIN);

  rawDiff_MR = rawPos_MR - lastRawPos_MR;
  lastRawDiff_MR = rawPos_MR - lastLastRawPos_MR;
  rawOffset_MR = abs(rawDiff_MR);
  lastRawOffset_MR = abs(lastRawDiff_MR);

  lastLastRawPos_MR = lastRawPos_MR;
  lastRawPos_MR = rawPos_MR;

  if((lastRawOffset_MR > flipThresh_MR) && (!flipped_MR)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff_MR > 0) {        // check to see which direction the drive wheel was turning
      flipNumber_MR--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber_MR++;              // ccw rotation
    }
    flipped_MR = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped_MR = false;
  }
   updatedPos_MR = rawPos_MR + flipNumber_MR*OFFSET_MR; // need to update pos based on what most recent offset is 

  ts_MR = 0.0127 * updatedPos_MR - 11.475;
  ts_MR = (ts_MR * 3.141592) / 180;
  return ts_MR;
}

// Updates position of Hall Effect sensor
float getPositionHallEffect(void){
  rawPos = int(angleSensor.getRawRotation());

  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  updatedPos = long(rawPos + flipNumber * OFFSET);

  ts = -0.0016*updatedPos - 2.7719;
  ts = (ts * 3.141592) / 180;
  ts = ts + 0.13;
  return ts;
}

// Map angles to pen x position
float pen_x(float theta_1, float theta_5){
  // find P2, and P4
  float x_2 = a1*cos(theta_1);
  float y_2 = a1*sin(theta_1);
  float x_4 = a4*cos(theta_5) - a5;
  float y_4 = a4*sin(theta_5);
  float x_h = 0.5*(x_2 + x_4);
  float y_h = 0.5*(y_2 + y_4);
  float dist_24 = sqrt(pow((x_2 - x_4), 2) + pow((y_2 - y_4), 2));
  float dist_3h = sqrt(pow(a2,2) - 0.25*pow(dist_24, 2));
  float holder = dist_3h / dist_24;
  float x_3 = x_h + holder *(y_4 - y_2);

  return x_3;
}

// Map angles to pen y position
float pen_y(float theta_1, float theta_5){
  float x_2 = a1*cos(theta_1);
  float y_2 = a1*sin(theta_1);
  float x_4 = a4*cos(theta_5) -a5;
  float y_4 = a4*sin(theta_5);
  float x_h = 0.5*(x_2 + x_4);
  float y_h = 0.5*(y_2 + y_4);
  float dist_24 = sqrt(pow((x_2 - x_4), 2)+ pow((y_2 - y_4), 2));
  float dist_3h = sqrt(pow(a2,2) - 0.25*pow(dist_24, 2));
  float holder = dist_3h / dist_24;
  float y_3 = y_h - holder*(x_4 - x_2);

  return y_3;
}

// Bound and scale pen position to game canvas in x
float find_x_Cursor(float x_3){
  float x_box = max(-4.0, min(0.75, x_3));
  return CANVAS_WIDTH/4.75*(x_box + 1.625) + CANVAS_WIDTH/2.0; 
}

// Bound and scale pen position to game canvas in y
float find_y_Cursor(float y_3){
  float y_box = max(6.0, min(10.5, y_3));
  return CANVAS_WIDTH/4.5*(y_box - 8.25) + CANVAS_WIDTH/2.0;
}

// Vibrate Motors
void vibrateMotors(int onBit){
  const int VIBRATE_STRENGTH = 1000;
  if(onBit){
    analogWrite(PWM_PIN1,VIBRATE_STRENGTH);
    digitalWrite(DIR_PIN1,vibeDirState);
    analogWrite(PWM_PIN2,VIBRATE_STRENGTH);
    digitalWrite(DIR_PIN2,vibeDirState);
  }else{
    analogWrite(PWM_PIN1,0);
    analogWrite(PWM_PIN2,0);
  }
}
