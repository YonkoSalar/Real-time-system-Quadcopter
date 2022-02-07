#include <Arduino_LSM9DS1.h>
#include <Servo.h>

// The order of the values in different measurements
#define YAW      0
#define PITCH    1
#define ROLL     2


// States for the state machine
#define STOPPED  0
#define STARTING 1
#define STARTED  2

int status = STOPPED;

// Useful since the IMU library uses degrees and arduino uses radians
#define RAD2DEG 180/PI
#define DEG2RAD PI/180


// --------- IMU and controller ------------------------- //
volatile float throttle = 0, yaw = 1500, roll = 1500, pitch = 1500;

// Gyro is in deg/sec //
// Acc is in Gs //

// Read values from the sensors
float ax, ay, az; 
float gx, gy, gz;

float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;

// The corrected orientation speed (gyro)
float angular_motions[3] = {0, 0, 0};

// The corrected orientation (combined gyro and acc)
float measures[3] = {0, 0, 0};

// Constants for the filters used for the accelerometer and gyro
const float HighPassFilterCoeff = 0.994;
const float LowPassFilterCoeff = 0.7;

// Max values the controller can tilt the drone in
// Currently only yaw is used, but pitch and roll should be implemented instead of the current way it is calculated
const float maxYaw = 30, maxPitch = 10, maxRoll = 10;

unsigned long emergencytimer = 0; // Used to measure time between last recieved signal from controller

// --------- Variables for motors -------------- //
volatile unsigned long pulse_length_esc1 = 0,
                       pulse_length_esc2 = 0,
                       pulse_length_esc3 = 0,
                       pulse_length_esc4 = 0;



// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll

// Errors
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]

// PID coefficients
float Kp[3] = {0.4, 0.4, 0.4}; // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3] = {0.0, 0.0, 0.0}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3] = {0, 30, 30};     // D coefficients in that order : Yaw, Pitch, Roll

// Used for mixing the throttle and PID values together
// Could be left out if the code is rewritten and new values for the PID are given
const float mixing = 0.5;



Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  // For displaying debugging to the serial monitor
  Serial.begin(9600);

  // For the HC-12
  Serial1.begin(9600);

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Calibrate the IMU
  calibrateGyro();
  calibrateAcc();

  // Attach the motors to servo library with operating pulses between 1000 us to 2000 us
  motor1.attach(D2, 1000, 2000);
  motor2.attach(D3, 1000, 2000);
  motor3.attach(D4, 1000, 2000);
  motor4.attach(D5, 1000, 2000);

  // A little delay to make the IMU a bit more stable after powering the arduino
  delay(2000);
}

void loop() {
  // 1. Read all the available sensors
  readSensor();

  // 1.5 Make sure the contact isnt lost
  if (millis() - emergencytimer > 105) { // Should be over 10 times each sec.
    status = STOPPED;
    stopAll();
  }

  // 2. Calculate the current angles from gyro & accelerometer
  computeAttitude();
  
  // 3. Calculate the target values for the PID controllers
  calculateSetPoints();

  // 4. Calculate the errors between the target and actual for the PID
  calculateErrors();

  // 5. Calculate the motor speeds with the PID
  if (isStarted()) {
    pidController();
  }
  
  // 6. Apply the motor speeds
  applyMotorSpeeds();
}

void readSensor() {
  // Check if all the values from the controller has been recieved
  if (Serial1.available() > 8) {
    emergencytimer = millis();
    // The controller will send 170 at the beginning of every transmission
    // This could be done better with checksums or control bytes
    if (Serial1.read() != 170) {
      stopAll();
      status = STOPPED;
    }
    // The controller sends 2 bytes for each, but serial read only reads 1 byte
    throttle =  (Serial1.read() << 8) | Serial1.read();
    yaw =       (Serial1.read() << 8) | Serial1.read();
    roll =      (Serial1.read() << 8) | Serial1.read();
    pitch =     (Serial1.read() << 8) | Serial1.read();

    // map the values from the controller to the esc value range
    throttle =  fmap(throttle, 0, 65535, 1000, 2000);
    yaw =       fmap(yaw, 0, 65535, 1000, 2000);
    roll =      fmap(roll, 0, 65535, 1000, 2000);
    pitch =     fmap(pitch, 0, 65535, 1000, 2000);
  }

  // Read accelerometer and gyro if available
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
}

void computeAttitude() {
  /*---------Calculate acc----------------*/
  // Apply offsets for acc
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;

  float rollAngle = -RAD2DEG * atan((float)ay / az);

  float pitchAngle = RAD2DEG * atan((float)ax / az);

  /*---------Calculate gyro----------------*/
  // Apply offsets for gyro

  gx -= gx_offset;
  gy -= gy_offset;
  //gz -= gz_offset; // Doesnt work. Why? ¯\_(ツ)_/¯

  /*--------------Combine acc and gyro----------*/;

  // To dampen the pitch and roll angles a complementary filter is used
  // Essentialy we use the gyro values and correct them with the accelerometer values
  // 0.008 is approxemately the time the loop uses
  measures[ROLL]  = applyComplementaryFilter(measures[ROLL], gx, rollAngle, 0.008);
  measures[PITCH] = applyComplementaryFilter(measures[PITCH], gy, pitchAngle, 0.008);
  measures[YAW]   = -gz + gz_offset; // Since the accelerometer is useless for the yaw, we dont use a complementary filter

  // Apply low-pass filter
  // We want to use the gyro values by themselves, but they are very noisy
  angular_motions[ROLL]  = applyLowPassFilter(angular_motions[ROLL], gx);
  angular_motions[PITCH] = applyLowPassFilter(angular_motions[PITCH], gy);
  angular_motions[YAW]   = applyLowPassFilter(angular_motions[YAW], measures[YAW]);

}

// A complementary filter
float applyComplementaryFilter(float _angularPos, float _gyroRaw,
                               float _angleDegrees, double _loopTime) {
  return HighPassFilterCoeff * (_angularPos + _gyroRaw * _loopTime)
         + (1 - HighPassFilterCoeff) * _angleDegrees;
}

// A low-pass filter
float applyLowPassFilter(float motion, float gyro){
  return LowPassFilterCoeff * motion + (1-LowPassFilterCoeff) * gyro;
}

// Absolutely needed, since the gyro drifts
// Get the drift of the gyro when initializing the drone
// The drift should not change too much during the little time the drone is operated
void calibrateGyro() {
  Serial.println("Starting calibration");
  int max_samples = 2000;

  for (int i = 0; i < max_samples; i++) {
    while (!IMU.gyroscopeAvailable());
    IMU.readGyroscope(gx, gy, gz);

    gx_offset += gx;
    gy_offset += gy;
    gz_offset += gz;
    //Serial.println("gx: " + String(gx) + "\tgy: " + String(gy) + "\tgz: " + String(gz));
  }

  // Calculate average offsets
  gx_offset /= max_samples;
  gy_offset /= max_samples;
  gz_offset /= max_samples;

  Serial.println("gx offset: " + String(gx_offset) + "\tgy offset: " + String(gy_offset) + "\tgz offset: " + String(gz_offset));
}

// Calibrate the accelerometer, since it is not entirely level.
// If the quadcopter is initialized tilted this will give bad values.
// Not really needed for our project, since the IMU is placed properly
// and since the accelerometer doesnt drift
void calibrateAcc() {
  Serial.println("Starting calibration");
  int max_samples = 200;

  for (int i = 0; i < max_samples; i++) {
    while (!IMU.accelerationAvailable());
    IMU.readAcceleration(ax, ay, az);

    ax_offset += ax;
    ay_offset += ay;
    az_offset += az;
  }

  // Calculate average offsets
  ax_offset /= max_samples;
  ay_offset /= max_samples;
  az_offset /= max_samples;
  
  az_offset -= 1; // To account for gravity
  
  Serial.println("ax offset: " + String(ax_offset) + "\tay offset: " +
                 String(ay_offset) + "\t az offset: " + String(az_offset));
}

void pidController() {
  // Initialize motor commands with throttle
  pulse_length_esc1 = throttle;
  pulse_length_esc2 = throttle;
  pulse_length_esc3 = throttle;
  pulse_length_esc4 = throttle;

  // Do not calculate anything if throttle is 0
  if (throttle > 1000) {
    // PID = e.Kp + ∫e.Ki + Δe.Kd
    float yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
    float pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
    float roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL] * Kd[ROLL]);

    // Keep values within acceptable range.
    yaw_pid   = minMax(yaw_pid, -400, 400);
    pitch_pid = minMax(pitch_pid, -400, 400);
    roll_pid  = minMax(roll_pid, -400, 400);

    // Calculate pulse duration for each ESC
    pulse_length_esc1 = throttle - roll_pid * mixing - pitch_pid * mixing + yaw_pid * mixing;
    pulse_length_esc2 = throttle + roll_pid * mixing - pitch_pid * mixing - yaw_pid * mixing;
    pulse_length_esc3 = throttle - roll_pid * mixing + pitch_pid * mixing - yaw_pid * mixing;
    pulse_length_esc4 = throttle + roll_pid * mixing + pitch_pid * mixing + yaw_pid * mixing;
  }

  // Prevent out-of-range-values
  pulse_length_esc1 = minMax(pulse_length_esc1, 1050, 2000);
  pulse_length_esc2 = minMax(pulse_length_esc2, 1050, 2000);
  pulse_length_esc3 = minMax(pulse_length_esc3, 1050, 2000);
  pulse_length_esc4 = minMax(pulse_length_esc4, 1050, 2000);

}


void resetPidController() {
  errors[YAW]   = 0;
  errors[PITCH] = 0;
  errors[ROLL]  = 0;

  error_sum[YAW]   = 0;
  error_sum[PITCH] = 0;
  error_sum[ROLL]  = 0;

  previous_error[YAW]   = 0;
  previous_error[PITCH] = 0;
  previous_error[ROLL]  = 0;
}

// Get the target values for the PID
void calculateSetPoints() {
  pid_set_points[YAW]   = fmap(yaw, 1000, 2000, -maxYaw, maxYaw);
  pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pitch);
  pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], roll);

}

float calculateSetPoint(float angle, int channel_pulse) {
  float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
  float set_point    = 0;

  // Need a dead band of 16µs for better result
  if (channel_pulse > 1508) {
    set_point = channel_pulse - 1508;
  } else if (channel_pulse <  1492) {
    set_point = channel_pulse - 1492;
  }

  set_point -= level_adjust;
  set_point /= 3;

  return set_point;
}

// Map function for floats, since the default arduino map function is for ints
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Calculate the errors used in the pid
void calculateErrors() {
  // Calculate current errors
  errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
  errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
  errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];

  // Calculate sum of errors : Integral coefficients
  error_sum[YAW]   += errors[YAW];
  error_sum[PITCH] += errors[PITCH];
  error_sum[ROLL]  += errors[ROLL];
  
  // Calculate error delta : Derivative coefficients
  delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
  delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
  delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

  // Save current error as previous_error for next time
  previous_error[YAW]   = errors[YAW];
  previous_error[PITCH] = errors[PITCH];
  previous_error[ROLL]  = errors[ROLL];
}


// Clamp the value
float minMax(float value, float min_value, float max_value) {
  if (value > max_value) {
    return max_value;
  } else if (value < min_value) {
    return min_value;
  }
  return value;
}


bool isStarted() {
  // When left stick is moved in the bottom left corner
  if (status == STOPPED && pitch == 1000 && throttle == 1000) {
    Serial.println("STARTING");
    status = STARTING;
  }

  // When left stick is moved back in the center position
  if (status == STARTING && throttle == 1000 && pitch >= 1900) {
    status = STARTED;
    Serial.println("STARTED FROM STOPPED");
    // Reset PID controller's variables to prevent bump start
    resetPidController();
  }

  // When left stick is moved in the bottom right corner
  if (status == STARTED && yaw >= 1900 && roll >= 1900) {
    status = STOPPED;
    Serial.println("STOPPED");
    // Make sure to always stop motors when status is STOPPED
    stopAll();
  }

  // Returns true if the PID should be calculated so that the motors will spin
  return status == STARTED;
}

// I find it better to set the ESC pulse to 0 rather than 1000 when the motors are stopped
// It seems like there is less power wasted on standby this way.
void stopAll() {
  pulse_length_esc1 = 0;
  pulse_length_esc2 = 0;
  pulse_length_esc3 = 0;
  pulse_length_esc4 = 0;
}

// Update the servo objects speeds
void applyMotorSpeeds() {
  motor1.writeMicroseconds(pulse_length_esc1);
  motor2.writeMicroseconds(pulse_length_esc2);
  motor3.writeMicroseconds(pulse_length_esc3);
  motor4.writeMicroseconds(pulse_length_esc4);
}
