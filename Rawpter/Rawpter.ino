#include <SPI.h>
#include <WiFiNINA.h>          //WiFi
#include <Arduino_LSM6DSOX.h>  //IMU get from the Arduino IDE Library Manager
#include <Servo.h>             //get from the Arduino IDE Library Manager

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//
//EASYCHAIR is used to put in some key dummy variables so you can sit in the easy chair with just the Arduino on a cord wiggling it around and seeing how it responds.
//Set EASYCHAIR to false when you are wanting to install it in the drone.
#define EASYCHAIR false

//The LOOP_TIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.  So, the loop time is set a little longer so the IMU has time to update from the control change.
#define LOOP_TIMING 100

// Add these defines near the top of the file with other defines
#define THROTTLE 0
#define ROLL 1  
#define ELEVATION 2
#define RUDD 3
#define THROTTLE_CUT 4

//The battery alarm code handles 14.8 or 7.4V LIPOs.  Set to your type of battery. If not hooked up, it will pull down and beep often.
#define BATTERYTYPE 14.8
#define BUZZER_PIN 9
int batteryVoltage = 777;             //just a default for the battery monitoring routine
unsigned long next_voltage_check = 0;  //used in loopBuzzer to check for voltage on the main battery.
bool beeping = false;         //For tracking beeping when the battery is getting low.

//Radio failsafe values for every channel in the event that bad reciever data is detected.
//These are for it to stay stable and descend safely versus totally cutting throttle and drop like a rock.
unsigned long PWM_throttle_zero = 1000;         //used when we want to take throttle to zero.  Failsafe is something higher as it is expected that failsafe is a value needed to safely land.
unsigned long PWM_throttle_fs = 1000;           //throttle  will allow it to descend to the ground if you adjust the throttlecutswitch_fs to 2000
unsigned long PWM_roll_fs = 1500;               //ail pretty much in the middle so it quits turning
unsigned long PWM_elevation_fs = 1500;          //elev
unsigned long PWM_rudd_fs = 1500;               //rudd
unsigned long PWM_ThrottleCutSwitch_fs = 1000;  //SWA less than 1300, cut throttle - must config a switch to Channel 5 in your remote.
bool UPONLYMODE = false;                        //UPONLYMODE is used to take the radio out of it other than thrust.  The intent is to get it tuned so it can at least take off.  Should be false once tuned.
float stick_dampener = 0.1;                     //0.1-1 Lower=slower, higher=noiser default 0.7

bool failsafed = false;
//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = -0.03;
float AccErrorY = 0.00;
float AccErrorZ = 0.01;
float GyroErrorX = 0.37;
float GyroErrorY = -0.04;
float GyroErrorZ = -0.45;

float Gyro_filter = .97;
float Accel_filter = .97;

float B_madgwick = 0.02;  //(default 0.04)
float q0 = 1.0f;          //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Controller parameters (this is where you "tune it".  It's best to use the WiFi interface to do it live and then update once its tuned.):
float i_limit = 60;   //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 18.0;   //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 18.0;  //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 10.0;    //Max yaw rate in deg/sec (default 160.0)
float maxMotor = 0.8;

float Kp_roll_angle = 10;  //Roll P-gain
float Ki_roll_angle = 8;  //Roll I-gain
float Kd_roll_angle = 2;  //Roll D-gain

float Kp_pitch_angle = 15;  //Pitch P-gain
float Ki_pitch_angle = 8;   //Pitch I-gain
float Kd_pitch_angle = 2;  //Pitch D-gain

float Kp_yaw = 0;  //Yaw P-gain default 30
float Ki_yaw = 0;  //Yaw I-gain default 5
float Kd_yaw = 0;  //Yaw D-gain default .015 (be careful when increasing too high, motors will begin to overheat!)

//========================================================================================================================//
//                                                     DECLARE PINS                                                       //
//========================================================================================================================//
//Radio:
//used so I quit getting confused begween the ARduino pins and the Radio Channel pins.
const int stickRightHorizontal = 2;  //right horizontal stick
const int stickRightVertical = 3;    //right vertical stick
const int stickLeftVertical = 4;     //left vertical stick
const int stickLeftHorizontal = 5;   //left horizontal stick
const int SwitchA = 6;               //SWA switch

const int throttlePin = stickLeftVertical;  //throttle - up and down on the
const int rollPin = stickRightHorizontal;   //ail (roll)
const int upDownPin = stickRightVertical;   //ele
const int ruddPin = stickLeftHorizontal;    //rudd
const int throttleCutSwitchPin = SwitchA;   //gear (throttle cut)

int throttleCutCounter = 0;
int throttleNotCutCounter = 0;
//Motor Electronic Speed Control Modules (ESC):
const int motorPins[] = {10, 15, 16, 14}; // m1Pin, m2Pin, m3Pin, m4Pin
Servo motorPWM[4]; // m1PWM, m2PWM, m3PWM, m4PWM

//========================================================================================================================//
//DECLARE GLOBAL VARIABLES
//========================================================================================================================//

//General stuff for controlling timing of things
float deltaTime;
float invFreq = (1.0 / LOOP_TIMING) * 1000000.0;

unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
bool throttle_is_cut = true;  //used to force the pilot to manually set the throttle to zero after the switch is used to throttle cut

//Radio communication:
unsigned long PWM_values[5]; // throttle, roll, elevation, rudd, throttleCutSwitch
unsigned long PWM_prev[4]; // throttle_prev, roll_prev, elevation_prev, rudd_prev 

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float motor_command_scaled[4]; // m1_command_scaled, m2_command_scaled, etc
int motor_command_PWM[4]; // m1_command_PWM, m2_command_PWM, etc

int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
unsigned long buzzer_millis;
unsigned long buzzer_spacing = 10000;

//WiFi Begin
int keyIndex = 0;
int status = WL_IDLE_STATUS;
bool ALLOW_WIFI = true;

WiFiServer server(80);
//WiFi End

//========================================================================================================================//
//BEGIN THE CLASSIC SETUP AND LOOP
//========================================================================================================================//

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  analogReadResolution(12); //The RP2040 has a 12 bit ADC versus the standard 10 from the Uno
  setupSerial();
  if (ALLOW_WIFI) setupWiFi();  //At first power on, a WiFi hotspot is set up for talking to the drone. (SSID Rawpter, 12345678)
  setupDrone();
  setupBatteryMonitor();

  digitalWrite(LEDG, HIGH); //GREEN
}

void loop() {
  tick();  //stamp the start time of the loop to keep our timing to 100Hz.  See tock() below.
  if (throttle_is_cut) 
  {
    //We are only going to allow these to slow down the loop if the throttle is cut.  Otherwise, the loop speed is too irregular.
    loopBuzzer();
    if (ALLOW_WIFI) loopWiFi();
  }
  loopDrone();
  tock();
}

void setupDrone() {
  // Initialize motor pins
  for(int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    motorPWM[i].attach(motorPins[i], 1060, 1860);
  }
  
  delay(5);

  radioSetup();
  setToFailsafe();
  PWM_values[0] = PWM_throttle_zero; // Set throttle to zero on startup

  IMUinit();

  // Early return if IMU calibration needed
  if (getRadioPWM(1) > 1800 && getRadioPWM(1) < 2400 && !EASYCHAIR) {
    calibrateESCs();
    return;
  }

  // Initialize motor commands to zero
  for(int i = 0; i < 4; i++) {
    motor_command_PWM[i] = 0;
  }

  // Early return if waiting for throttle down
  if (getRadioPWM(1) > 1060 && getRadioPWM(1) < 2400 && !EASYCHAIR && getRadioPWM(5) < 1300) {
    delay(1000);
    return;
  }
}

void loopDrone() {
  getIMUdata();                                            //Pulls raw gyro andaccelerometer data from IMU and applies LP filters to remove noise
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);  //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  getDesiredAnglesAndThrottle();                           //Convert raw commands to normalized values based on saturated control limits
  PIDControlCalcs();                                       //The PID functions. Stabilize on angle setpoint from getDesiredAnglesAndThrottle
  controlMixer();                                          //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands();                                         //Scales motor commands to 0-1
  throttleCut();                                           //Directly sets motor commands to off based on channel 5 being switched
  Troubleshooting();                                       //will do the print routines that are uncommented
  commandMotors();                                         //Sends command pulses to each ESC pin to drive the motors
  getRadioSticks();                                        //Gets the PWM from the radio receiver
  failSafe();                                              //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
}

void setupBatteryMonitor() 
{
  buzzer_millis = millis();
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(A6, INPUT);
}

void loopBuzzer() 
{  //this monitors the battery.  the lower it gets, the faster it beeps.
  unsigned long myTime=millis();
  if (!beeping) {
    if (myTime - buzzer_millis > (buzzer_spacing)) {
      digitalWrite(BUZZER_PIN, HIGH);
      beeping = true;
      buzzer_millis = myTime;
    }
  } else {
    if (myTime - buzzer_millis > 80) {
      beeping = false;
      buzzer_millis = myTime;
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  if (myTime > next_voltage_check) {
    next_voltage_check = myTime + 10000;  //checkvoltage once every 10 seconds versus every loop.    
    batteryVoltage = analogRead(A6);
    if (BATTERYTYPE == 14.8) {
      //based on 330K and 51K voltage divider that takes 16.8V to 2.25V
      if (batteryVoltage > 1457) buzzer_spacing = 40000;
      else if (batteryVoltage > 1440 ) buzzer_spacing = 30000;
      else if (batteryVoltage > 1400) buzzer_spacing = 20000;
      else if (batteryVoltage > 1350) buzzer_spacing = 10000;
      else if (batteryVoltage > 1301) buzzer_spacing = 500;
      else buzzer_spacing = 100;
    } else {
      //Depends on your resistors and battery choice
    }
  }
}

void printJSON() {
  String tick = createJSON();
  Serial.println(tick);
}

void Troubleshooting() {
  if (current_time - print_counter < 34000) return;  //Don't go too fast or it slows down the main loop
  print_counter = micros();

  //Print data at 100hz (uncomment one at a time for troubleshooting)
  //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands(); //Prints the values being written to the motors (expected: 1000 to 2000)
  //printtock();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations and less the Gyro/Acc update speed.  Set by LOOP_TIMING)
  // #if EASYCHAIR
    printJSON();
  // #endif
}

void setupSerial() {
  Serial.begin(2000000);
  delay(100);
}

void tick() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  deltaTime = (current_time - prev_time) / 1000000.0;  //division takes it from micros to seconds.  1000000 ms=1 second
}
//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

/**
 * Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
 */
void controlMixer() {
  // Early return if invalid inputs
  if (!thro_des || !roll_PID || !pitch_PID || !yaw_PID) {
    return;
  }

  // Mixing coefficients for each motor
  const float mixTable[4][4] = {
    // thro, pitch, roll, yaw
    {maxMotor, -1, 1, 1},  // Motor 1 (Front Left)
    {maxMotor, -1, -1, -1}, // Motor 2 (Front Right) 
    {maxMotor, 1, -1, 1},   // Motor 3 (Back Right)
    {maxMotor, 1, 1, -1}    // Motor 4 (Back Left)
  };

  // Calculate mixed output for each motor
  for(int i = 0; i < 4; i++) {
    motor_command_scaled[i] = mixTable[i][0] * thro_des + 
                             mixTable[i][1] * pitch_PID +
                             mixTable[i][2] * roll_PID + 
                             mixTable[i][3] * yaw_PID;
    motor_command_scaled[i] = constrain(motor_command_scaled[i], 0, 1.0);
  }
}

/**
 * Initialize IMU
 */
void IMUinit() {
  if (!IMU.begin()) {
    while (true)
      ;
  }
  delay(15);
}

/**
 * Request full dataset from IMU
 */
void getIMUdata() {
  //Accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccY = AccY - AccErrorZ;
  } 
  // else return;
  if (IMU.gyroscopeAvailable()) {
    //Gyro
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
  }
  // if (IMU.temperatureAvailable())
  // {
  //   int temperature_deg = 0;
  //   IMU.readTemperature(temperature_deg);

  //   Serial.print("LSM6DSOX Temperature = ");
  //   Serial.print(temperature_deg);
  //   Serial.println(" °C");
  // }
}

/**
 * Normalizes desired control values to appropriate values
 */
void getDesiredAnglesAndThrottle() {
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in degrees
   * yaw_des is scaled to be within max yaw in degrees/sec.
   */
  thro_des = (PWM_values[THROTTLE] - 1000.0) / 1000.0;   // throttle
  roll_des = (PWM_values[ROLL] - 1500.0) / 500.0;    // roll
  pitch_des = (PWM_values[ELEVATION] - 1500.0) / 500.0;   // elevation
  yaw_des = (PWM_values[RUDD] - 1500.0) / 500.0;     // rudd

  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0);                //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;     //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch;  //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;        //Between -maxYaw and +maxYaw
}

/**
 * Computes control commands based on state error (angle)
 */
void PIDControlCalcs() {
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesiredAnglesAndThrottle(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */


  if (PWM_values[THROTTLE] < 1030)
  { //This will keep the motors from spinning with the throttle at zero should the drone be sitting unlevel.
    integral_roll_prev = 0;
    integral_pitch_prev = 0;
    error_yaw_prev = 0;
    integral_yaw_prev = 0;
    roll_PID=0;
    pitch_PID=0;
    yaw_PID=0;
    return;
  }
  if (UPONLYMODE)
  {
    //For troubleshooting.  UPONLYMODE can be set with the web interface.
    roll_des = 0;
    pitch_des = 0;
    yaw_PID = 0;
  }

  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * deltaTime;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Limit integrator to prevent saturating
  derivative_roll = GyroX; //(roll_des-roll_IMU-roll_des-previous_IMU)/dt=current angular velocity since last IMU read and therefore GyroX in deg/s
  roll_PID = 0.0001 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll);  //Scaled by .0001 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * deltaTime;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  derivative_pitch = GyroY;
  pitch_PID = .0001 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch);  //Scaled by .0001 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ versus angle.  In other words, your stick is setting y axis rotation speed - not the angle to get to.
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * deltaTime;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
  derivative_yaw = (error_yaw - error_yaw_prev) / deltaTime;
  yaw_PID = .0001 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //Scaled by .0001 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev = integral_roll;
  integral_pitch_prev = integral_pitch;
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

/**
 * Scale normalized actuator commands to values for ESC protocol
 */
void scaleCommands() {
  for(int i = 0; i < 4; i++) {
    motor_command_PWM[i] = motor_command_scaled[i] * 180;
  }
}

/**
 * Get raw PWM values for every channel from the radio
 */
void getRadioSticks() {
  rcUpdate();

  // Get raw values
  PWM_values[THROTTLE] = getRadioPWM(1);
  PWM_values[ROLL] = getRadioPWM(2);
  PWM_values[ELEVATION] = getRadioPWM(3);
  PWM_values[RUDD] = getRadioPWM(4);
  PWM_values[THROTTLE_CUT] = getRadioPWM(5);

  // Early return if values invalid
  if (!validatePWMValues()) {
    return;
  }

  // Apply filters
  PWM_values[THROTTLE] = filterThrottle(PWM_values[THROTTLE], PWM_prev[THROTTLE]);
  for(int i = ROLL; i <= RUDD; i++) {
    PWM_values[i] = filterStick(PWM_values[i], PWM_prev[i]);
  }

  // Update previous values
  for(int i = THROTTLE; i <= RUDD; i++) {
    PWM_prev[i] = PWM_values[i];
  }
}

void setToFailsafe() {
  PWM_values[THROTTLE] = PWM_throttle_fs;
  PWM_values[ROLL] = PWM_roll_fs;
  PWM_values[ELEVATION] = PWM_elevation_fs;
  PWM_values[RUDD] = PWM_rudd_fs;
  PWM_values[THROTTLE_CUT] = PWM_ThrottleCutSwitch_fs;
}

/**
 * If radio gives garbage values, set all commands to default values
 */
void failSafe() {
  /*
   * Radio connection failsafe used to check if the getRadioSticks() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */


  //Triggers for failure criteria
  //unsigned minVal = 800;
  //unsigned maxVal = 2200;
  //if (PWM_ThrottleCutSwitch>maxVal || PWM_ThrottleCutSwitch<minVal || PWM_throttle > maxVal || PWM_throttle < minVal || PWM_roll > maxVal ||PWM_roll < minVal || PWM_Elevation > maxVal || PWM_Elevation < minVal || PWM_Rudd > maxVal || PWM_Rudd < minVal)


  if (PWM_values[THROTTLE] < 800 || PWM_values[THROTTLE] > 2200) {  // Check throttle
    failsafed = true;
    setToFailsafe();
  } else {
    failsafed = false;
  }
  
  #if EASYCHAIR
    PWM_values[THROTTLE] = 1500;  //For testing in the easy chair
  #endif
}

/**
 * Send pulses to motor pins
 */
void commandMotors() {
  for(int i = 0; i < 4; i++) {
    motorPWM[i].write(motor_command_PWM[i]);
  }
}

/**
 * Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
 */
void calibrateESCs() {
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero.
   */
  for(int i = 0; i < 4; i++) {
    motorPWM[i].write(180);
  }
  delay(5000);
  for(int i = 0; i < 4; i++) {
    motorPWM[i].write(0);
  }
  delay(5000);
}

/**
 * Directly set actuator outputs to minimum value if triggered
 * Monitors the state of radio command PWM_throttle and directly sets the mx_command_PWM values to minimum (1060 is
 * minimum , 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function 
 * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
 * the motors to anything other than minimum value. Safety first. 
 */
void throttleCut() {
  #if EASYCHAIR 
    throttle_is_cut = false;
    return;
  #endif

  if (throttle_is_cut) {
    killMotors();
    return;
  }

  if (PWM_values[THROTTLE_CUT] < 1300) {
    if (++throttleCutCounter > 10) {
      killMotors();
    }
    return;
  }

  if (throttle_is_cut && PWM_values[THROTTLE_CUT] > 1500) {
    if (PWM_values[THROTTLE] < 1040 && ++throttleNotCutCounter > 10) {
      throttle_is_cut = false;
      throttleNotCutCounter = 0;
      throttleCutCounter = 0;
    } else {
      killMotors();
    }
    return;
  }

  //Handle when things are going upside down
  if (roll_IMU > 55 || roll_IMU < -55 || pitch_IMU > 55 || pitch_IMU < -55) {
    if (++throttleCutCounter > 4) {
      killMotors();
    }
    return;
  }
  
  throttleNotCutCounter = 0;
  throttleCutCounter = 0;
}

/**
 * sets the PWM to its lowest value to shut off a motor such as whne the throttle cut switch is fliped.
 */
void killMotors() {
  throttle_is_cut = true;
  throttleCutCounter = 10;
  
  for(int i = 0; i < 4; i++) {
    motor_command_PWM[i] = 0;
  }
}

/**
 * Regulate main loop rate to specified frequency in Hz
 */
void tock() {
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. I have matched 
   * it just below the Gyro update frequency.  invFreq is set at the top of the code.
   */
  //Sit in loop until appropriate time has passed while checking for any WiFi client activity.
  while (invFreq > (micros() - current_time)) {
    if (ALLOW_WIFI&&throttle_is_cut) loopWiFi();
  };
}

float invSqrt(float x) {
  return 1.0 / sqrtf(x);  //board is fast enough to just take the compute penalty lol suck it arduino nano
}

// Helper function to filter throttle
float filterThrottle(float current, float prev) {
  if (current - prev < 0) {
    return 0.95 * prev + 0.05 * current; // Going down - slow
  }
  return stick_dampener * prev + (1 - stick_dampener) * current; // Going up - fast
}

// Helper function to filter other sticks
float filterStick(float current, float prev) {
  return (1.0 - stick_dampener) * prev + stick_dampener * current;
}

// Helper function to validate PWM values
bool validatePWMValues() {
  for(int i = 0; i < 5; i++) {
    if (PWM_values[i] < 800 || PWM_values[i] > 2200) {
      return false;
    }
  }
  return true;
}
