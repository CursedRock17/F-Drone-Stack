//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick

*/
//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//

// Uncomment only one receiver type
//#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX
#define USE_CRSF_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

#define USE_MPU6050_I2C  // Default
#define USE_MATEK3901    // Default

//========================================================================================================================//

// REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer
//#include <odometry_node.h>


#if defined USE_SBUS_RX
  #include "src/SBUS/SBUS.h"   //sBus interface
#endif

#if defined USE_DSM_RX
  #include "src/DSMRX/DSMRX.h"
#endif

#if defined USE_CRSF_RX
  #include <CRSFforArduino.hpp>
#endif

#if defined USE_MPU6050_I2C
  #include "src/MPU6050/MPU6050.h"
  MPU6050 mpu6050;
#else
  #error No IMU defined...
#endif

#if defined USE_MATEK3901 
  #include "src/MATEK3901/MATEK3901.h"
  Matek3901 matek;
#else 
  #error No Optical Flow Sensor defined...
#endif

//==========================================================================//
//                             USER-SPECIFIED VARIABLES                     //
//==========================================================================//
// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

// Setup gyro and accel full scale value selection and scale factor
#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#endif

#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif

// Filter parameters - Defaults tuned for 2kHz loop rate;
float B_madgwick = 0.04;  // Madgwick filter parameter
float B_accel = 0.14;     // Accelerometer LP filter paramter (default: 0.14)
float B_gyro = 0.1;       // Gyro LP filter paramter (default: 0.1)

// Position Controller Low Pass Filter Values 
float A_positionX = 0.2; // LPF horizontal (X) filter
float A_positionY = 0.2; // LPF forward (Y) filter
float A_positionZ = 0.2; // LPF altitude (Z) filter

// Optical Flow Sensor Constants
const float FLOW_SCALE_X = -800.0f / 1000.0f;
const float FLOW_SCALE_Y = -800.0f / 1000.0f;
// Range of valid data from the VL53L0X, given by Matek themselves [8cm -> 200cm]
const float RANGEFINDER_INITIAL_HEIGHT_M = 0.0368f + 0.095f;
const float MAX_RANGE_M = 2.0f;
const float MIN_RANGE_M = 0.08f;

// IMU calibration parameters -
// calibrate IMU using calculate_IMU_error() in the void setup() to get these
// values, then comment out calculate_IMU_error()
float AccErrorX = 0.04;
float AccErrorY = 0.00;
float AccErrorZ = 0.05;
float GyroErrorX = -2.09;
float GyroErrorY = 0.29;
float GyroErrorZ = -1.17;

// OpFlow calibration parameters
float rangeErrorZ = 0;

// Controller parameters (take note of defaults before modifying!):
float i_limit = 25;     // Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 24.0;     // Max rohover_throtll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxPitch = 24.0;    // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 128.0;     // Max yaw rate in deg/sec

float Kp_roll_angle = 0.12;    //Roll P-gain - angle mode
float Ki_roll_angle = 0.18;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.03;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.54;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.12;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.18;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.03;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.54;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.0075;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.0025;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.00001;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.0075;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.0025;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.00001; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.18;           //Yaw P-gain
float Ki_yaw = 0.03;          //Yawcrsf.upd I-gain
float Kd_yaw = 0.00009;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

// Optical Flow Controller Params
float Kp_positionX = 0.1;  // Horizontal (X-position) Proportional Gain
float Ki_positionX = 0.0;  // Horizontal (X-position) Integral Gain
float Kd_positionX = 0.0;  // Horizontal (X-position) Derivative Gain

float Kp_positionY = 0.1;  // Forward (Y-position) Proportional Gain
float Ki_positionY = 0.0;  // Forward (Y-position) Integral Gain
float Kd_positionY = 0.0;  // Forward (Y-position) Derivative Gain

float Kp_positionZ = 0.18;  // Altitude (Z-position) Proportional Gain
float Ki_positionZ = 0.03;  // Altitude (Z-position) Integral Gain
float Kd_positionZ = 0.00009;  // Altitude (Z-position) Derivative Gain

float maxZ = 2.0;  // Maximum position on the Z-Axis in Meters

// Drone Characteristics
float hover_throttle = 0.4;
float drone_weight_kg = 0.097 + 0.005 + 0.104;

// Simulation Package
struct SensorPacket {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float range;
};

union SensorSerial {
  SensorPacket simulatedPkt;
  unsigned char serialStream[sizeof(SensorPacket)];
};

const unsigned char pktHeader[2] = {0xAA, 0x55};
const unsigned char pktFooter = 0xFE;

const bool simulatedEnvironment = true;
SensorSerial simulatedSensors;

// Radio failsafe values for every channel in the event that bad reciever
// data is detected. Recommended defaults:
// Defined: Throttle, Ail, Elevation, Rudder, Arm 1, Aux2
// TODO: Add parameters.yaml file to allow user to set
unsigned long thro_range[3] = {1194, 1500, 1807};  // Amount of Power
unsigned long ail_range[3]  = {1194, 1500, 1807};  // Roll 
unsigned long ele_range[3]  = {1194, 1500, 1807};  // Pitch 
unsigned long rud_range[3]  = {1194, 1500, 1807};  // Yaw
unsigned long arm_range[2]  = {1000, 1792};        // Off/On
unsigned long channel_fs[6] = { thro_range[0], ail_range[1], ele_range[1], rud_range[1], 
                                arm_range[0], 1000 };

//==========================================================================//
//                                DECLARE PINS                              //
//==========================================================================//

// Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)
// LED Pin : 13
// IMU Pins : SCL = 19, SDA = 18
// CRSF Pins: RX = 15, TX = 14
// Optical Flow Pins: RX = 16, TX = 17
//// Pinout Meanings:     throttle, ail, elevation, rudd, gear, aux1
// TODO: Add extra pins for certain flight modes using handset
const int channelPins[6] = {15,     16,  17,        20,    21,  22};
const int PPM_Pin = 23;

// OneShot125 ESC pin outputs (Had to go in reverse):
// Motor 1, 2, 3, 4
const int mPin[4] = {1, 2, 3, 4};

//==========================================================================//
//                            GLOBAL VARIABLES                              //
//==========================================================================//

// General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

// Radio communication:
unsigned long channel_pwm [6];
unsigned long channel_pwm_prev[4];

#if defined USE_CRSF_RX
  CRSFforArduino * crsf = nullptr;
  const int crsfChannels = 5;  // Move as needed based on Handheld
#endif
#if defined USE_SBUS_RX
  SBUS sbus(Serial3);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
  DSM1024 DSM;
#endif

// IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

// Optical Flow Position Information:
float positionX_cur, positionY_cur, positionZ_cur = 0.0f;
float positionX_prev, positionY_prev, positionZ_prev = 0.0f;

// Optical Flow Twist Information:
float current_velocityX = 0.0f;
float current_velocityY = 0.0f;
float current_velocityZ = 0.0f;

float current_angular_velocityX = 0.0f;
float current_angular_velocityY = 0.0f;
float current_angular_velocityZ = 0.0f;

// Position Hold Controller:
bool valid_flow;

// X Values for all the various control methods
float error_positionX, error_positionX_prev = 0;
float positionX_des_prev, integral_positionX = 0;
float integral_positionX_il, integral_positionX_ol = 0;
float integral_positionX_prev, integral_positionX_prev_il, integral_positionX_prev_ol = 0;
float derivative_positionX, positionX_PID = 0;

// Y Values for all the various control methods
float error_positionY, error_positionY_prev = 0;
float positionY_des_prev, integral_positionY = 0;
float integral_positionY_il, integral_positionY_ol = 0;
float integral_positionY_prev, integral_positionY_prev_il, integral_positionY_prev_ol = 0;
float derivative_positionY, positionY_PID = 0;

// Z Values for all the various control methods
float error_positionZ, error_positionZ_prev = 0;
float positionZ_des_prev, integral_positionZ = 0;
float integral_positionZ_il, integral_positionZ_ol = 0;
float integral_positionZ_prev, integral_positionZ_prev_il, integral_positionZ_prev_ol = 0;
float derivative_positionZ, positionZ_PID = 0;

// Position Hold Desired States
float positionX_des, positionY_des, positionZ_des;
float positionX_passthru, positionY_passthru, positionZ_passthru;

// Mixer (Motors)
float m_command_scaled[4];
int m_command_PWM[4];

// Flight status
enum DroneState {
  DORMANT = 0,       // Not Armed, not Ready
  ARMED = 1,         // No Throttle, but Ready
  FLYING = 2,        // Armed and Given Throttle, Changing Position
  POSITION_HOLD = 3,  // No additional Throttle, Hold Position
  CRASHED = 4         // If we have hit something and cannot operate
};
DroneState droneState;

//==========================================================================//
//                                 VOID SETUP                               //
//==========================================================================//

void setup() {
  Serial.begin(115200); //USB serial
  delay(500);

  // Initialize all pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify
  for (int i = 0; i < 4; i++)
  {
    pinMode(mPin[i], OUTPUT);
  }
  //Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  //Initialize radio communication
  radioSetup();

  // Setup all MicroROS connections
  //odom_setup();

  //Set radio channels to default (safe) values before entering main loop
  for (int i = 0; i < 6; i++)
  {
    channel_pwm[i] = channel_fs[i];
  }
  // Initialize IMU communication
  IMUinit();
  // Initialize Optical Flow communication
  opticalFlowInit();

  delay(50);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
  //calculateRangefinderError();

  //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  // Code will not proceed past here if this function is uncommented!

  // Command ARM OneShot125 ESC from 125 to 250us pulse length
  for (int i = 0; i < 4; i++)
  {
    m_command_PWM[i] = 125;
  }
  armMotors(); //Loop over commandMotors() until ESCs happily arm

  // Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)
}



//==========================================================================//
//                               MAIN LOOP                                  //
//==========================================================================//

void loop() {
  // Keep track of current time and how much time has elapsed since last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0f;

  // Get status of our drone, to see what it's currently doing
  droneStateUpdate();

  // If we're using a Simulated environment, we need replicated sensor values 
  if (simulatedEnvironment) {
    readSimulatedPeripherals();
    // Fuse our simulated IMU
    Madgwick(simulatedSensors.simulatedPkt.gx, -simulatedSensors.simulatedPkt.gy, -simulatedSensors.simulatedPkt.gz, 
            -simulatedSensors.simulatedPkt.ax, simulatedSensors.simulatedPkt.ay, simulatedSensors.simulatedPkt.az, dt);
    // Fuse our simulated optical flow
    positionZ_cur = simulatedSensors.simulatedPkt.range;
  } else {
    // Get vehicle state with RAW data 
    // Use Madgwick to update the rpy
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);
  
    // Utilize the Optical Flow + Rangefinder for Pose information
    opticalFlowUpdate();
  }

  // Compute desired state
  // Convert raw commands to normalized values based on saturated control limit
  getDesState();

  // Orientation PID Controller - SELECT ONE:
  controlANGLE();    // Stabilize on angle setpoint
  //controlANGLE2(); // Stabilize on angle setpoint using cascaded method.
  
  // Over time, the battery depletes or payload changes, so the hover throttle should
  // be able to change over time as well.
  calculateHoverThrottle();

  // Pose PID Controller
  positionHold();

  // Actuator mixing and scaling to PWM values
  controlMixer();  // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands();  // Scales motor commands to 125 to 250 range (OneShot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  // Throttle cut check
  throttleCut();  // Q sets motor commands to low based on Armed (ch5)

  // Command actuators
  commandMotors();  // Sends command pulses to each motor pin using OneShot125
  writeMotorPWM();

  // Autonomous Code - all looped code should run here (i.e publishers)
  odometryUpdate();

  //printMotorCommands();
  //printAltitudeOutput();
  //printOpticalFlowOutput();

  // Get vehicle commands for next loop iteration
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  // Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */

  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  // Don't let integrator build if throttle is too low
  if (buildIntegral())
  {
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt;
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2() {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt;
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt;
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol + GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt;
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlRATE() {
    //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (buildIntegral()) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void positionHold() {
  // DESCRIPTION: Computes control commands based on state error (position)
  /* Simple PID loop which takes the current position on all 3 axis from the rangefinder and optical
   * flow sensors. This data may be preprocessed via an Extended Kalman Filter or similar. It may
   * then be fused with other sensors like a camera, more rangefinders, sonar, etc. It compares
   * the desired transformation state with the current state to get an error, this error is then 
   * compared to previous errors and the time it's taken between errors. Each of these states 
   * represent the proportional, integral, and derivative errors which each have a respective gain 
   * value, which must tuned extremely well.
   */
  
  // X-Axis (Horizontal)
  error_positionX = positionX_des - positionX_cur;
  integral_positionX = integral_positionX_prev + error_positionX*dt;
  // Don't let integrator build if throttle is too low
  if (buildIntegral())
  {
    integral_positionX = 0;
  }
  // Saturate integrator to prevent unsafe buildup
  integral_positionX = constrain(integral_positionX, -i_limit, i_limit);
  derivative_positionX = (positionX_cur - positionX_prev) / dt;
  positionX_PID = (Kp_positionX*error_positionX + 
                        Ki_positionX*integral_positionX - 
                        Kd_positionX*derivative_positionX); 

  // Update X-Position variables
  integral_positionX_prev = integral_positionX;
  positionX_prev = positionX_cur;
  
  // Y-Axis (Forward)
  error_positionY = positionY_des - positionY_cur;
  integral_positionY = integral_positionY_prev + error_positionY*dt;
  // Don't let integrator build if throttle is too low
  if (buildIntegral())
  {
    integral_positionY = 0;
  }
  // Saturate integrator to prevent unsafe buildup
  integral_positionY = constrain(integral_positionY, -i_limit, i_limit);
  derivative_positionY = (positionY_cur - positionY_prev) / dt;
  positionY_PID = (Kp_positionY*error_positionY + 
                        Ki_positionY*integral_positionY - 
                        Kd_positionY*derivative_positionY); 

  // Update Y-Position variables
  integral_positionY_prev = integral_positionY;
  positionY_prev = positionY_cur;
  
  // Z-Axis (Altitude)
  error_positionZ = positionZ_des - positionZ_cur;
  integral_positionZ = integral_positionZ_prev + error_positionZ*dt;
  // Don't let integrator build if throttle is too low
  if (buildIntegral())
  {
    integral_positionZ = 0;
  }
  // Saturate integrator to prevent unsafe buildup
  integral_positionZ = constrain(integral_positionZ, -i_limit, i_limit);
  derivative_positionZ = (positionZ_cur - positionZ_prev) / dt;
  positionZ_PID = (Kp_positionZ*error_positionZ + 
                        Ki_positionZ*integral_positionZ - 
                        Kd_positionZ*derivative_positionZ); 

  // Update Z-Position variables
  integral_positionZ_prev = integral_positionZ;
  positionZ_prev = positionZ_cur;
}


void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands()
   * in preparation to be sent to the motor ESCs and servos.
   *
   *Relevant variables:
   *positionZ_PID - ranging on the Z-axis based on hover throttle
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   */

  // Quad mixing - in "X" Format - Remeber these are 1-indexed so subtract 1
  /*
    Front
    1   3
      X
    2   4 
    Back       - Battery Cables
  */

  // Need to figure out what hover throttle is based on the weight of the drone
  // Lift Throttle: Thrust = ~2 * mg
  // Hover Throttle: Thrust = mg
  m_command_scaled[0] = hover_throttle + thro_des + positionZ_PID - pitch_PID 
    + roll_PID + yaw_PID;  // Front Left
  m_command_scaled[1] = hover_throttle + thro_des + positionZ_PID + pitch_PID 
    + roll_PID - yaw_PID;  // Back Left
  m_command_scaled[2] = hover_throttle + thro_des + positionZ_PID - pitch_PID
    - roll_PID - yaw_PID;  // Front Right
  m_command_scaled[3] = hover_throttle + thro_des + positionZ_PID + pitch_PID
    - roll_PID + yaw_PID;  // Back Right
}

void droneStateUpdate() {
  // DESCRIPTION: Values of throttle and armed to update drone state
  if (channel_pwm[4] < 1500)
  {
    droneState = DORMANT;
  } else if ((channel_pwm[4] >= 1500)) {
    droneState = ARMED;
    // If the channel is not armed, there's no reason to check throttle values
    // Otherwise, let's see the state of our drone.
    if (abs(thro_des) >= 0.05) {
      droneState = FLYING;
    }
    if (abs(thro_des) < 0.05 && (positionZ_cur > RANGEFINDER_INITIAL_HEIGHT_M)) {
      droneState = POSITION_HOLD;
    }
  }
}

void IMUinit() {
  // DESCRIPTION: Initialize IMU
  Wire.begin();
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

  mpu6050.initialize();

  if (mpu6050.testConnection() == false) {
    //Serial.println("MPU6050 initialization unsuccessful");
    //Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer data
  /*
   * Reads accelerometer, gyro, data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ.
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  // LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  // Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  // LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
}

void calculate_IMU_error() {
  // DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;

  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #endif

    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;

    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");

  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot.
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time)/1000000.0;
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void opticalFlowInit()
{
  // DESCRIPTION: Create instantation of our Matek3901 Optical Flow Sensor
  //Serial.println(F("\n Matek 3901-L0X MSPv2 bring-up"));

  // Start the sensor UART
  matek.begin(Serial4, 115200);
  matek.setLPFAlpha(0.4f);
  // matek.enableLogging(&Serial);
}

void opticalFlowUpdate()
{
  // DESCRIPTION: Grab the values from our Optical Flow Sensor for Pose estimation
  // Drain UART and parse MSP frames
  matek.poll();

  // If new data arrived, compute simple body-frame velocity estimate
  if (matek.getFresh()) {
    const float range_m = matek.getLPFRangeM();
    const float fx = matek.getLPFFlowX();
    const float fy = matek.getLPFFlowY();

    if (range_m >= MIN_RANGE_M && range_m <= MAX_RANGE_M) {
      // For small angles, velocity ~ (flow * range) * scale
      current_velocityX = FLOW_SCALE_X * fx * range_m;
      current_velocityY = FLOW_SCALE_Y * fy * range_m;
      positionZ_cur = range_m + rangeErrorZ;
      valid_flow = true;
    } else {
      valid_flow = false;
    }
  }

  //  Add a velocity-damping term to roll/pitch commands:
  //
  //   roll_cmd  += K_vx * (-current_velocityX);
  //   pitch_cmd += K_vy * (+current_velocityY);
  //  
  // - For a simple "pos hold", integrate vx,vy into a tiny position error
  //   accumulator with clamping and drive it with a PI onto roll/pitch.
}

  void calculateRangefinderError()
  {
    // DESCRIPTION: At startup, try to remove error in the Rangefinder based on overall position
    int c = 0;
    // Read Rangefinder values 1000 times
    rangeErrorZ = 0;
    while (c < 1000) {
      matek.poll();
      if (matek.getFresh()) {
        const float range = matek.getLPFRangeM();
        rangeErrorZ += (range - RANGEFINDER_INITIAL_HEIGHT_M);
        c++;
      }
    }

    rangeErrorZ /= c;
    //Serial.print("float rangeErrorZ = ");
    //Serial.println(rangeErrorZ);
  }

void calculateHoverThrottle()
{
  // return hoverThrottle;
}

void getDesState() 
{
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_pwm[0] - 1500.0)/614.0; //Between -0.5 and 0.5
  roll_des = (channel_pwm[1] - 1500.0)/307.0; //Between -1 and 1
  pitch_des = (channel_pwm[2] - 1500.0)/307.0; //Between -1 and 1
  yaw_des = (channel_pwm[3] - 1500.0)/307.0; //Between -1 and 1

  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5

  //Constrain within normalized bounds
  thro_des = constrain(thro_des, -0.5, 0.5);  // Between -1 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw

  // Check to see if we want to change altitude
  if (droneState == FLYING) {
    positionZ_des = positionZ_cur;
    positionZ_des = constrain(positionZ_des, 0, maxZ);  // Between 0 and +maxZ
  }

  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors().
   */
  // Scaled to 125us - 250us for OneShot125 protocol
  for (int i = 0; i < 4; i++)
  {
    m_command_PWM[i] = (m_command_scaled[i] * 125) + 125;
  }

  // Constrain commands to motors within OneShot125 bounds
  for (int i = 0; i < 4; i++)
  {
    // Throttle should start at a very slow speed
    m_command_PWM[i] = constrain(m_command_PWM[i], 125, 225);
  }
 }

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
   */

  #if defined USE_PPM_RX || defined USE_PWM_RX
    for (int i = 0; i < 6; i++)
    {
      channel_pwm[i] = getRadioPWM(i);
    }

  #elif defined USE_SBUS_RX
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      const float scale = 0.615;
      const float bias  = 895.0;
      for (int i = sbus 0; i < 6; i++)
      {
        channel_pwm[i] = sbusChannels[i] * scale + bias;
      }
    }

  #elif defined USE_CRSF_RX
    crsf->update();  // Check Callback Function in RadioComm file

  #elif defined USE_DSM_RX
    if (DSM.timedOut(micros())) {
        Serial.println("*** DSM RX TIMED OUT ***");
    }
    else if (DSM.gotNewFrame()) {
        uint16_t dsm_values[num_DSM_channels];
        DSM.getChannelValues(dsm_values, num_DSM_channels);

        for (int i = 0; i < 6; i++)
        {
          channel_pwm[i] = dsm_values[i];
        }
    }
  #endif

  // Low-pass the critical commands and update previous values
  float b = 0.7; // Lower=slower, higher=noiser
  for (int i = 0; i < 4; i++)
  {
    channel_pwm[i] = (1.0 - b)*channel_pwm_prev[i] + b*channel_pwm[i];
    channel_pwm_prev[i] = channel_pwm[i];
  }
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2800;
  bool check_fs = false;

  // Triggers for failure criteria
  for (int i = 0; i < 6; i++)
  {
    if ((channel_pwm[i] > maxVal) || (channel_pwm[i] < minVal))
      check_fs = true;
  }

  // If any failures, set to default failsafe values
  if (check_fs)
  {
    for (int i = 0; i < 6; i++)
    {
      channel_pwm[i] = channel_fs[i];
    }
  }
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, OneShot125 protocol
  /*controlMixer
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM[4] = {0, 0, 0, 0};

  //Write all motor pins high
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(mPin[i], HIGH);
  }
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 4 ) { //Keep going until final (4th) pulse is finished, then done
    timer = micros();
    for (int i = 0; i < 4; i++)
    {
      if ((m_command_PWM[i] <= timer - pulseStart) && (flagM[i] == 0)) {
        digitalWrite(mPin[i], LOW);
        wentLow = wentLow + 1;
        flagM[i] = 1;
      }
    }
  }
}

void armMotors() {
  //DESCRIPTION: Sends many commcalibrateand pulses to the motors, to be used to arm motors in the void setup()
  /*
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
   while (true) {
      prev_time = current_time;
      current_time = micros();
      dt = (current_time - prev_time)/1000000.0;

      digitalWrite(13, HIGH); //LED on to indicate we are not in main loop

      getCommands(); //Pulls current available radio commands
      failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
 
      for (int i = 0; i < 4; i++)
      {
        m_command_scaled[i] = thro_des;
      }

      scaleCommands(); //Scales motor commands to 125 to 250 range (OneShot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
      //throttleCut(); //Directly sets motor commands to low based on state of ch5
      commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol

      //printRadioData();
      //printMotorCommands();
      //printDesiredState();

      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //Maximum param bound desired, increase param by diffParam for each loop iteration
    param += diffParam;
  } else if (state == 0) { //Minimum param bound desired, decrease param by diffParam for each loop iteration
    param -= diffParam;
  }

  param = constrain(param, param_min, param_max); //Constrain param within max bounds

  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*
   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency
   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()
   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical
   *  statements in order to fade controller gains, for example between the two dynamic configurations.
   *
   */
  if (param > param_des) { //Need to fade down to get to desired
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param -= diffParam;
  } else if (param < param_des) { //Need to fade up to get to desired
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param += diffParam;
  }

  param = constrain(param, param_lower, param_upper); //Constrain param within max bounds

  return param;
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_pwm[5] and directly sets the mx_command_PWM values to minimum (120 is
      minimum for OneShot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_pwm[5] is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_pwm[5] is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if (droneState == DORMANT) {
    for (int i = 0; i < 4; i++)
    {
      m_command_PWM[i] = 120;
    }
  }
}

void odometryUpdate()
{
  float orientation[4] = {q0, q1, q2, q3};
  float position[3] = {1.0, 2.0, 3.0};
  float linear_velocity[3] = {0.0, 0.0, 0.0};
  float angular_velocity[3] = {0.0, 0.0, 0.0};
  //odom_pub_callback(orientation, position, linear_velocity, angular_velocity);
}

void writeMotorPWM()
{
  // DESCRIPTION: Send the motor PWM values to the Serial port for simulation
  // *Note* must be in the form of a byte
  Serial.write((uint8_t *)m_command_PWM, sizeof(m_command_PWM));
}

void readSimulatedPeripherals()
{
  // DESCRIPTION: Read the "fake" peripheral values from sim to update drone
  // Use a State Based Enum to control index movement
  static enum { WAIT_HEADER_FIRST, WAIT_HEADER_SECOND, READ_PAYLOAD, WAIT_FOOTER } state = WAIT_HEADER_FIRST;
  static size_t index = 0;

  while (Serial.available()) {
    unsigned char currentByte = Serial.read();
    if (state == WAIT_HEADER_FIRST) {
        if (currentByte == pktHeader[0]) {
          state = WAIT_HEADER_SECOND;
        } 
    } else if (state == WAIT_HEADER_SECOND) {
        if (currentByte == pktHeader[1]) {
          index = 0;
          state = READ_PAYLOAD;
        } else {
          // back to start if not correct sequence
          state = WAIT_HEADER_FIRST;
        }
    } else if (state == READ_PAYLOAD) {
      simulatedSensors.serialStream[index++] = currentByte;
        if (index >= sizeof(SensorPacket)) {
          state = WAIT_FOOTER;
        }
    } else if (state == WAIT_FOOTER) {
      if (currentByte == pktFooter) {
          simulatedSensors.serialStream[index] = currentByte;
          index++;
        }
        // Regardless, reset to look for next header
        state = WAIT_HEADER_FIRST;
    }
  }
}

void simulationReset()
{
// DESCRIPTION: Reset all of our desired and given values when we crash/hit the reset button
// Quaternions
q0 = 1.0f; 
q1 = 0.0f;
q2 = 0.0f;
q3 = 0.0f;

// Controller:
error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

// Normalized desired state:
thro_des, roll_des, pitch_des, yaw_des = 0;
roll_passthru, pitch_passthru, yaw_passthru = 0;

// Optical Flow Position Information:
positionX_cur, positionY_cur, positionZ_cur = 0.0f;
positionX_prev, positionY_prev, positionZ_prev = 0.0f;

// Optical Flow Twist Information:
current_velocityX = 0.0f;
current_velocityY = 0.0f;
current_velocityZ = 0.0f;
current_angular_velocityX = 0.0f;
current_angular_velocityY = 0.0f;
current_angular_velocityZ = 0.0f;


}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    // Channels Index From 1 Not 0
    for (int i = 0; i < (1 + crsfChannels); i++) {
      Serial.print(F("CH: "));
      Serial.print(channel_pwm[i]);
      Serial.print(F(" "));
    }
    Serial.println(F("\n"));
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
    Serial.print(F(" Desired Z:"));
    Serial.println(positionZ_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F(" AccY:"));
    Serial.print(AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(AccZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printOpticalFlowOutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Range (in m): "));
    Serial.println(positionZ_cur);
  }
}

void printAltitudeOutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print("Current_Height:");
    Serial.print(positionZ_cur);
    Serial.print("Desired_Height:");
    Serial.println(positionZ_des);
    Serial.print("Z_PID:");
    Serial.print(positionZ_PID);
    Serial.print("Motor_1_Response");
    Serial.println(m_command_PWM[0]);
  }
}

void printPIDOutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(yaw_PID);
    Serial.print(F(" positionZ_PID:"));
    Serial.print(positionZ_PID);
    Serial.print(F(" Desired Throttle:"));
    Serial.println(thro_des);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    for (int i = 0; i < 4; i++)
    {
      Serial.print(F("m_command"));
      Serial.print(i);
      Serial.print(F(": "));
      Serial.print(m_command_PWM[i]);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}

//=========================================================================================//

// HELPER FUNCTIONS
float invSqrt(float x) {
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

bool buildIntegral()
{
  bool shouldBuild = true;
  if (channel_pwm[0] > 1470 && channel_pwm[0] < 1530) {
    shouldBuild = false;
  }
  return shouldBuild;
}
