// Includes
#include <math.h>
#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define E_Value 2.71828     // Value for e
#define g       9.81        // gravitational constant

bool firstContact = 1;
// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

double rs = 0.073152;   //[m]
double rp = 0.004191;   //[m]

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;

// Kinematics variables
double xh = 0;           // Position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;
double pitch_prev;          // pitch of the mass at previous time step
double pitch_prev2;
double theta_dot;           // Velocity of the handle
double dTheta;
double dTheta_dot;
double theta_dot_prev;
double theta_dot_prev2;
double theta_dot_filt;         // Filtered angular velocity of the mass
double theta_dot_filt_prev;
double theta_dot_filt_prev2;
double m = 0.25; 
double kp;
double kd; 
double xh_max = .05;
double xh_min = -.05;

//Parameters for MSD sim
double m_msd = 2;
double b_msd = 10;
double k_msd = 300;
double k_user = 1000;

// position with the position of the mass
double dt = 0.0006;      // time step used for simulation
double x_msd = 0;        // position of the mass
double v_msd = 0;        // velocity of the mass
double a_msd = 0;        // acceleration of the mass
double xi_msd = 10;   // initial position of the mass
double vi_msd = 0;       // initial velocity of the mass
double ai_msd = 0;       // initial acceleration of the mass

int counter = 0;

// Force output variables
double force = 0;                 // Force at the handle
double Tp = 0;                    // Torque of the motor pulley
double duty = 0;                  // Duty cylce (between 0 and 255)
unsigned int output = 0;          // Output command to the motor

BNO080 IMU;
float roll;
float pitch;
float yaw;

//These pins can be any GPIO
byte imuCSPin = 10;
byte imuWAKPin = 9;
byte imuINTPin = 3;
byte imuRSTPin = 2;

// --------------------------------------------------------------
// Setup function
// --------------------------------------------------------------
void setup()
{
  // Set up serial communication
  Serial.begin(115200);
  
  if(IMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin) == false)
  {
    Serial.println("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...");
    while(1);
  }
  Serial.println("BNO080 Connected over SPI");
  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A

  // Initialize motor
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

  IMU.enableRotationVector(50); //Send data update every 50ms
  //IMU.enableGyro(1);

  while (IMU.dataAvailable() == false){
    delay(1);
  }
  roll = (IMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
  pitch = (IMU.getPitch());
  yaw = (IMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts and mass angle ****
  //*************************************************************
  if (IMU.dataAvailable() == true){
    roll = (IMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    pitch = (IMU.getPitch());
    yaw = (IMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
  }

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if ((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiff > 0) {       // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  updatedPos = rawPos + flipNumber * OFFSET; // need to update pos based on what most recent offset is


  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // Define kinematic parameters you may need
  double rh = 0.075;   //[m]
  
  // Step B.6
  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double theta_s = .0106 * updatedPos - 9.0251; // m = 0.0131 [deg/pos], b = -8.504 [deg]

  // Step B.7
  // Compute the position of the handle based on the angle of the sector pulley
  xh = rh * (theta_s * PI / 180);

  // Step B.8

  // Calculate the velocity of the handle
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9 * dxh + 0.1 * dxh_prev;

  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;

  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;

  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;

  //*************************************************************
  //** Section 3. Assign a motor output toque in Newton-Meters **
  //*************************************************************

  //*************************************************************
  //******************* Control Algorithms ********************
  //*************************************************************

  float theta_d = pitch*180/PI;  //compute desired angle based on gravity
  float theta_dot_d = 0; //compute desired velocity based on momentum

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  theta_dot_filt = .9 * theta_dot + 0.1 * theta_dot_prev;

  // Record the position and velocity
  pitch_prev2 = pitch_prev;
  pitch_prev = pitch;

  theta_dot_prev2 = theta_dot_prev;
  theta_dot_prev = theta_dot;

  theta_dot_filt_prev2 = theta_dot_filt_prev;
  theta_dot_filt_prev = theta_dot_filt;
   
  kp = .16;
  kd = 0.00007;
 
  if (xh < xh_min || xh > xh_max){
    Tp = -kp*(pitch) - kd*(dxh_filt/rh*180.0/PI);
  } else {
    Tp = -kp*(pitch) - kd*(dxh_filt/rh*180.0/PI);
  }

  //TODO: if mass out of range, Tp = 0;

  v_msd = v_msd + a_msd * dt;
  x_msd = x_msd + v_msd * dt;

  // calculate force and a_msd 
  // Determine whether the user's position is in front
  if (theta_s > x_msd)
  {
    //Serial.println(xh);
    if (firstContact){
      Tp = dxh_filt/abs(dxh_filt) * 10;
    }
    // to do: what is the force when the xh > x_msd
    force = -((k_user)*(theta_s-x_msd));
    // what is the acceleration of the msd? (include effect of contact with user as well as other forces)
    a_msd = (-force-k_msd*(x_msd-xi_msd)+(b_msd*v_msd))/m_msd;
  }
  else {
    // to do: what is the force when the xh < x_msd
    force = 0;
    // what is the acceleration of the msd? 
    a_msd = ((-b_msd*v_msd)-(k_msd*(x_msd-xi_msd)))/m_msd;
  }

//Communicate to processing
//using this counter is optional, just limits how many times we print to Processing (increasing baud rate can also help if need be)
  if (counter >= 100) {
    counter = 0;
    //print to processing
    Serial.print(theta_s, 4);
    Serial.print(' ');
    Serial.println(x_msd, 4);

  } else {
    counter++;
  }

  
  //*************************************************************
  //************* Section 4. Toque output ***********************
  //*************************************************************

  // Determine correct direction for motor torque
  if (Tp > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)hh
  duty = sqrt(abs(Tp) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
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
