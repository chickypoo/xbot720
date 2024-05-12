/***************************************************************************
 *                            Bitwise Definition                           *
 ***************************************************************************/
#define BIT0    0x01
#define BIT1    0x02
#define BIT2    0x04
#define BIT3    0x08
#define BIT4    0x10
#define BIT5    0x20
#define BIT6    0x40
#define BIT7    0x80
/***************************************************************************
 *                        Arduino Pinout Definition                        *
 ***************************************************************************/
#define TX      1   // HC-05 RX
#define RX      0   // HC-05 TX

#define RCHA    2   // Right Encoder Channel A
#define LCHA    3   // Left Encoder Channel A
#define RIN1    6   // Right Motor Enable 1
#define RIN2    5   // Right Motor Enable 2
#define LIN1    9   // Left Motor Enable 1
#define LIN2    10  // Left Motor Enable 2
/***************************************************************************
 *                           System Specification                          *
 ***************************************************************************/
#define WHEEL_DIA_MM 90
#define PI           3.1415
#define WHEEL_TRAVEL WHEEL_DIA_MM * PI
#define CPR          757       // Count per Revolution

#define SPEED_VARIANCE 5
#define DEFAULT_MIN_SPEED 100  // in mm/s
#define DEFAULT_MAX_SPEED 500  // in mm/s
#define MIN_SPEED         50   // in mm/s
#define MAX_SPEED         1000 // in mm/s

#define SPEED_INC_1       8
#define SPEED_INC_2       16
#define SPEED_DEC_1       8
#define SPEED_DECAY       0.9
/***************************************************************************
 *                            Static Receive Code                          *
 ***************************************************************************/
#define LDIR    BIT7           // Left Direction Bit
#define LS0     BIT6
#define LS1     BIT5
#define LS2     BIT4
#define RDIR    BIT3           // Right Direction Bit
#define RS0     BIT2
#define RS1     BIT1
#define RS2     BIT0

#define LSPD    (LS0 | LS1 | LS2)
#define RSPD    (RS0 | RS1 | RS2)

#define FASTEST BIT2
#define FAST    (BIT2 | BIT1)
#define NORMAL  BIT1
#define SLOW    (BIT1 | BIT0)
#define SLOWEST BIT0
#define STOP    0
#define ERRSPD  119

#define AD_CD1  63              // Special Admin Code to SET speed
#define AD_CD2  127             // Special Admin Code to ECHO speed
/***************************************************************************
 *                            Extra Specifications                         *
 ***************************************************************************/
#define LEFT    0
#define RIGHT   1
#define FWD     0
#define RWD     1
#define BRK     2

#define AVG_LEN 4         // Averager length
/***************************************************************************
 *                           Status Bit Definition                         *
 ***************************************************************************/
#define PCM     BIT0      // Calculate the Current Speed
#define CALC    BIT1      // Calculate the Target Speed
#define READ    BIT2      // Receive latest command from bluetooth

#define MODE_NORMAL   1        // Normal 5 speed configuration
#define MODE_VARIABLE 2        // Variable Speed configuration
/***************************************************************************
 *                                 Globals                                 *
 ***************************************************************************/
volatile unsigned int counts[2] = {0};         // Real time counter for Left Channel A
const float KP = 2.63;                         // Proportion Control
const float KI = 8.94;                         // Integral Control
const float KD = 0.51;                         // Derivative Control

float speedTotalError[2] = {0};                 // eIntegral for PID
float pSpeedError[2] = {0};                     // Previous Error for the Derivative Control

unsigned char code = 0;                         // Incoming code from Vision Controller
unsigned char mode; 

unsigned int arrCounts[2][AVG_LEN] = {0};       // Ongoing Averager for the count
size_t indexer = 0;                             // Iterator for the averager

float speedY[2] = {0};                          // Current Speed (from reading)
float speedR[2] = {0};                          // Speed Target
int power[2] = {0};                             // Power needed to drive for speed
unsigned char motorDir[2] = {0};                // Direction of motors

unsigned char status = 0;                       // Main loop status
unsigned int speed[SPEED_VARIANCE];             // This is the range of speed the car uses

void setup() {
  Serial.begin(57600);
  // Initiate the speed array
  setSpeed(DEFAULT_MIN_SPEED, DEFAULT_MAX_SPEED, SPEED_VARIANCE);
  // Setup the pinout
  pinMode(RIN1, OUTPUT);
  pinMode(RIN2, OUTPUT);
  pinMode(RCHA, INPUT);
  attachInterrupt(digitalPinToInterrupt(RCHA), rPulse, RISING);

  pinMode(LIN1, OUTPUT);
  pinMode(LIN2, OUTPUT);
  pinMode(LCHA, INPUT);
  attachInterrupt(digitalPinToInterrupt(LCHA), lPulse, RISING);
  // Set the mode of the car
  mode = MODE_VARIABLE;
  // Setup a timers
  cli();
  // Using Timer 2 because pin 11 and 13 is free
  TCCR2A = TCCR2B = 0;            // Clear register for Timer 2

  TCCR2B |= (BIT2 | BIT1 | BIT0); // Prescalar 1:1024
  TIMSK2 |= BIT1;                 // CC Timer 2A
  OCR2A = 125;                    // Timer 2 interrupt every 8 ms
  sei();
}

void loop() {
  // Using pulse counting method to check amount of counts in each main loop
  static unsigned long pt;
  static unsigned char receiverNotFound = 0;
  float currentCounts[2], dt;
  unsigned long t;

  if(status & PCM){
    // Turn off Interrupts to capture pulse counts
    noInterrupts();                             // Turn off interrupt to encoders
    arrCounts[LEFT][indexer] = counts[LEFT];    // Record the volatile counts
    arrCounts[RIGHT][indexer] = counts[RIGHT]; 
    counts[LEFT] = counts[RIGHT] = 0;           // Zero the counts
    indexer = (indexer + 1) % AVG_LEN;          // Increase the average index
    interrupts();                               // Turn interrupt back on
    // Averager through the pulse counter
    getAverageCounts(currentCounts);
    // Find the time difference delta T  
    t = millis();
    dt = (float)(t - pt) / 1000.0;
    pt = t;
    // Incremental speed target if it is variable speed mode
    if(mode == MODE_VARIABLE)
      setSpeedTarget();
    // Using the delta T and count difference, together with CPR and wheel size
    // Find the speed the wheel travels at
    calculateCurrentSpeed(currentCounts, dt);
    calculateNewPower(LEFT, dt);
    calculateNewPower(RIGHT, dt);
    setMotorPower();

    status &= ~PCM;
  } else if(status & CALC){
    // Use the code to decide the direction, and speed of each wheel
    updateTarget();
    updateDirection();
    status &= ~CALC;
  } else if(status & READ){   // Fetch code
    if(Serial.available()){
      code = Serial.read();
      receiverNotFound = 0;      
    } else {
      // failure to read 32 times (~5 seconds) will zero the code
      if(++receiverNotFound >= 32)
        code = receiverNotFound = 0;
    }
    status |= CALC;
    status &= ~READ;
  }  
}

/* setSpeedTarget
 * This is a variable speed functionality that incrementally increase the speed or decrease based on received code
 */
void setSpeedTarget(){
  // code 100 (4)= speed++
  // code 110 (3)= speed+
  // code 010 (2)= stay speed
  // code 011 (1)= speed-
  // code 001 (0)= speed%-
  // code 000 = speed 0
  size_t index[2];
  index[LEFT] = speedIndexer((code & LSPD) >> 4);
  index[RIGHT] = speedIndexer((code & RSPD));
  switchSpeed(LEFT, index[LEFT]);
  switchSpeed(RIGHT, index[RIGHT]);
}

/* switchSpeed
 * This is the auxiliary method for the setSpeedTarget
 * argument: side, the side of the wheel to configure
 * argument: ind, the index of the wheel determined from the code
 */
void switchSpeed(int side, size_t ind){
  switch(ind){
    case 4:
      if(speedR[side] < MAX_SPEED)
        speedR[side] = min(speedR[side] + SPEED_INC_2, MAX_SPEED);
      break;
    case 3:
      if(speedR[side] < MAX_SPEED)
        speedR[side] = min(speedR[side] + SPEED_INC_1, MAX_SPEED);
      break;
    case 2: break;
    case 1:
      if(speedR[side] > MIN_SPEED)
        speedR[side] = max(speedR[side] - SPEED_DEC_1, MIN_SPEED);
      break;
    case 0:
      if(speedR[side] > MIN_SPEED)
        speedR[side] = max(speedR[side] * SPEED_DECAY, MIN_SPEED);
      break;
    default: break;
  }
}

/* setMotorPower
 * This sets the motors from the calculated value in global
 */
void setMotorPower(){  
  setMotor(LEFT, motorDir[LEFT], power[LEFT]);
  setMotor(RIGHT, motorDir[RIGHT], power[RIGHT]);
}

/* calculateNewPower
 * This function calculates what the power should be at based on current speed and target speed
 * argument: side, LEFT or RIGHT side of the vehicle
 * argument: dt, time since last time the function is called in the main loop
 */
void calculateNewPower(unsigned char side, float dt){
  float speedE, speedD, speedU;

  speedE = speedR[side] - speedY[side];                       // Calculate E(s)
  speedTotalError[side] += speedE * dt;                       // Calculate eIntegral(s)
  speedD = (speedE - pSpeedError[side]) / dt;  // Calculate E'(s)

  speedU = KP * speedE + KI * speedTotalError[side] + KD * speedD; // Calculate U(s)

  pSpeedError[side] = speedE;                                 // Update deltaE
  if(speedY[side] == speedR[side])
    speedTotalError[side] = 0;

  power[side] = (int)(speedU / unitPVS(speedU));
  if(power[side] > 255)
    power[side] = 255;
  else if(power[side] < 0)
    power[side] = 0;
}

/* getAverageCounts
 * This function does the averager for the counts
 * argument: avgCurrentCounts, an array of counts
 */
void getAverageCounts(float * avgCurrentCounts){
  avgCurrentCounts[LEFT] = average(arrCounts[LEFT], AVG_LEN);
  avgCurrentCounts[RIGHT] = average(arrCounts[RIGHT], AVG_LEN);
}

/* calculateCurrentSpeed
 * This function calculate the speed of the wheels based on average counts
 * argument: currentCounts, average of the count pair
 * argument: dt, the time difference between each of the iteration
 */
void calculateCurrentSpeed(float * currentCounts, float dt){
  speedY[LEFT] = currentCounts[LEFT] / CPR * WHEEL_TRAVEL / dt;
  speedY[RIGHT] = currentCounts[RIGHT] / CPR * WHEEL_TRAVEL / dt;
}

/* updateTarget
 * This function changes the speed target of both wheel
 */
void updateTarget(){
  size_t index[2];
  static unsigned char pSpeedCode = 0;
  if(pSpeedCode == (code & (LSPD | RSPD)))
    return;
  // Byte shift to determine the speed code
  // Extract the index of the speed code
  index[LEFT] = speedIndexer((code & LSPD) >> 4);
  index[RIGHT] = speedIndexer((code & RSPD));
  // Set the Target Speed R(s)
  if(mode == MODE_NORMAL){
    if(index[LEFT] != ERRSPD)
      speedR[LEFT] = (index[LEFT] == SPEED_VARIANCE) ? 0 : speed[index[LEFT]];
    if(index[RIGHT] != ERRSPD)
      speedR[RIGHT] = (index[RIGHT] == SPEED_VARIANCE) ? 0 : speed[index[RIGHT]];
  } else if(mode == MODE_VARIABLE){
    // This mode only sets target speed to 0 if brake
    if(index[LEFT] == SPEED_VARIANCE)
      speedR[LEFT] = 0;
    if(index[RIGHT] == SPEED_VARIANCE)
      speedR[RIGHT] = 0;
  }

  pSpeedCode = (code & (LSPD | RSPD));
}

/* updateDirection
 * This function changes the direction of both wheel
 */
void updateDirection(){
  static unsigned char pDirectionCode = 0;
  if(pDirectionCode == (code & (LDIR | RDIR)))
    return;
  // Byte shift to determine the direction code
  // Extract the direction of the directional code
  motorDir[LEFT] = ((code & LDIR) >> 7);
  motorDir[RIGHT] = ((code & RDIR) >> 3);

  pDirectionCode = (code & (LDIR | RDIR));
}

/* speedIndexer
 * This returns the index of the corresponding speed from the code
 * argument: speedCode, the 3-bit speed code
 * return: the index of the speed
 */
size_t speedIndexer(unsigned char speedCode){
  switch(speedCode){
    case FASTEST: return 4;
    case FAST:    return 3;
    case NORMAL:  return 2;
    case SLOW:    return 1;
    case SLOWEST: return 0;
    case STOP:    return SPEED_VARIANCE;
    default:      return ERRSPD;
  }
}

/* setMotor
 * This is the manual way to set the motor speed by calling the setRightMotor and setLeftMotor
 * argument: side, the left or right wheel selection
 * argument: dir, the direction to rotate the wheel in order to achieve motion
 * argument: pwm, the speed value scaling from 0-255 of the 6V motor
 */
void setMotor(unsigned char side, unsigned char dir, unsigned char pwm){
  switch(side){
    case LEFT:
      setLeftMotor(dir, pwm);
      break;
    case RIGHT:
      setRightMotor(dir, pwm);
      break;
    default:
      break;
  }
}

/* setRightMotor
 * This is the manual way to set the right motor speed
 * argument: dir, the direction to rotate the wheel in order to achieve motion
 * argument: pwm, the speed value scaling from 0-255 of the 6V motor
 */
void setRightMotor(unsigned char dir, unsigned char pwm){
  switch(dir){
    case FWD:
      digitalWrite(RIN2, LOW);
      analogWrite(RIN1, pwm);
      break;
    case RWD:
      digitalWrite(RIN1, LOW);
      analogWrite(RIN2, pwm);
      break;
    case BRK:
      digitalWrite(RIN1, 0);
      digitalWrite(RIN2, 0);
      break;
    default:
      break;
  }
}

/* setLeftMotor
 * This is the manual way to set the left motor speed
 * argument: dir, the direction to rotate the wheel in order to achieve motion
 * argument: pwm, the speed value scaling from 0-255 of the 6V motor
 */
void setLeftMotor(unsigned char dir, unsigned char pwm){
  switch(dir){
    case FWD:
      digitalWrite(LIN2, LOW);
      analogWrite(LIN1, pwm);
      break;
    case RWD:
      digitalWrite(LIN1, LOW);
      analogWrite(LIN2, pwm);
      break;
    case BRK:
      digitalWrite(LIN1, 0);
      digitalWrite(LIN2, 0);
      break;
    default:
      break;
  }
}

/* setSpeed
 * This function sets the speed of the car with new values
 * argument: minSpeed, minimum speed of the car (in mm/s)
 * argument: maxSpeed, maximum speed of the car (in mm/s)
 * argument: amount, the different stages between the min and max speed
 */
void setSpeed(int minSpeed, int maxSpeed, int amount){
  int deltaSpeed = maxSpeed - minSpeed;
  int speedInc = deltaSpeed / (amount - 1);
  for(int i = 0; i < amount; i++){
    speed[i] = minSpeed + speedInc * i;
  }
}

/* rPulse
 * This function is for the interrupt for Right Encoder Channel A
 * It only increment the count from right wheel by 1
 */
void rPulse(){
  counts[RIGHT]++;
}

/* lPulse
 * This function is for the interrupt for Left Encoder Channel A
 * It only increment the count from left wheel by 1
 */
void lPulse(){
  counts[LEFT]++;
}

// Timer 2A ISR (runs at 8ms);
ISR(TIMER2_COMPA_vect){
  static unsigned char i = 0, si = 0;
  i++;
  
  if(i == 10)
    status |= PCM;    // Check Pulse every 8ms x 10 = 80ms of the 160ms
  if(i == 20){
    status |= READ;   // Check Serial import every 8ms x 10 = 80ms of the 160ms
    i = 0;
  }  
  TCNT2 = 0;
}

/* average
 * This computes the average of a given array
 * argument: a, the array of numerics to be averaged
 * argument: n, the size of array a
 */
float average(unsigned int * a, size_t n){
  unsigned int sum = 0;
  for(size_t i = 0; i < n; i++)
    sum += a[i];
  return (float)sum / n;
}

/* unitPVS
 * This function scales the given SPEED to a byte scale
 * argument: s, the speed of the given motor
 */
float unitPVS(float s){
  if(s > 1800) return 14.5;
  if(s > 900) return 14.0;
  if(s > 650) return 13.5;
  if(s > 410) return 12.75;
  if(s > 290) return 12.0;
  if(s > 170) return 10.5;
  return 8.5;
}
