#include "Arduino.h"

#include "time_utils.h"

// Analog pins connected to IMU
int PIN_Z_ACC = 3;
int PIN_Y_ACC = 4;
int PIN_X_RATE = 5;

// Motor control pins from motor control
// shield
int PIN_MOTOR_A_PWM = 3;
int PIN_MOTOR_B_PWM = 11;
int PIN_MOTOR_A_DIR = 12;
int PIN_MOTOR_B_DIR = 13;

#define PUBLISH_PERIOD 0.033

double last_update;
double start_time;
double last_publish_time;

struct CartpoleData {
  float y_acc;
  float z_acc;
  float x_rate;

  float y_acc_offset;
  float z_acc_offset;
  float x_rate_offset;

  // estimated angle from that
  float est_angle;
  float est_angle_vel;

  // From wheel odometry
  float est_pos;
  float drive_command;
  float drive_command_lp;

  float est_angle_pred;
  float est_angle_meas;

  float last_pot_setting; // defunct
  float zero_angle_setting;
  float zero_angle_adjusted;
};
CartpoleData data;

void send_IMU_struct() {
  Serial.write('S');
  Serial.write((uint8_t *)&data, sizeof(CartpoleData));
  Serial.write('E');
  return;
}


void doUpdateAccelerations(float dt){
  // 300 mv / g
  data.z_acc = 5. * ((float)analogRead(PIN_Z_ACC)) / 1024. / 0.3;
  data.y_acc = 5. * ((float)analogRead(PIN_Y_ACC)) / 1024. / 0.3;
/*
  if (data.z_acc_offset < 0.0){
    data.z_acc_offset = data.z_acc;
  }

  if (data.y_acc_offset < 0.0){
    data.y_acc_offset = data.y_acc;
  }
*/
  data.z_acc -= data.z_acc_offset;
  data.y_acc -= data.y_acc_offset;
}

float GYRO_OFFSET_RC = 10.0;
void doUpdateGyro(float dt){
  // 2mv / (deg / sec) sensitivity on the gyro
  data.x_rate = 5. * ((float)analogRead(PIN_X_RATE)) / 1024.;
  data.x_rate /= 0.002;
  data.x_rate *= 3.1415 / 180.;

  // Initialize offset if uninitialized
  if (data.x_rate_offset < 0.0){
    data.x_rate_offset = data.x_rate;
  } else {
    // Otherwise low-pass the gyro heavily for offset
    float alpha = dt / (dt + GYRO_OFFSET_RC);
    data.x_rate_offset = alpha * (data.x_rate) + (1. - alpha) * data.x_rate_offset;
  }

  data.x_rate -= data.x_rate_offset;
}

#define EST_ANGLE_VEL_RC 0.01
#define DRIVE_COMMAND_LP_RC 0.1
void doStateEstimate(float dt){
  doUpdateAccelerations(dt);
  doUpdateGyro(dt);

  // Get velocity straight from gyro, low-pass filtered a little
  float alpha = dt / (dt + EST_ANGLE_VEL_RC);
  data.est_angle_vel = (1. - alpha) * data.est_angle_vel + alpha * data.x_rate;

  // Estimate angle from IMU
  float new_angle_measured = atan2(data.z_acc, data.y_acc);
  // Estimate angle from last angle and gyro
  float new_angle_predicted = data.est_angle + dt * data.est_angle_vel;

  data.est_angle = new_angle_measured * 0.02 + new_angle_predicted * 0.98;
  data.est_angle_meas = new_angle_measured;
  data.est_angle_pred = new_angle_predicted;

  data.est_pos += dt * data.drive_command;
  alpha = dt / (dt + DRIVE_COMMAND_LP_RC);
  data.drive_command_lp = (1. - alpha) * data.drive_command_lp + alpha * data.drive_command;
}

#define SPEED_DEADZONE_HWIDTH 0.3
void setDriveCommand(float speed, bool linearize = true){
  // Bounded on range -1, 1
  speed = fmin(fmax(speed, -1.), 1.);
  float adjusted_speed = speed;
  if (linearize){
    // Apply adjustment to speed to counter the motor nonlinearities
    
    // Adjust #1: Motor has deadzone around zero.
    // Remove this by pushing speeds outward into the range
    // [DEADZONE_HWIDTH, 1.0]
    // This *doesn't* account for the hysteresis of the
    // stiction causing this problem -- this'll compensate
    // well for starting from stopped, but not compensate well
    // for decreasing speed when the wheel is already moving.
    if (adjusted_speed > 0){
      adjusted_speed = adjusted_speed * (1. - SPEED_DEADZONE_HWIDTH) + SPEED_DEADZONE_HWIDTH;
    } else {
      adjusted_speed = adjusted_speed * (1. - SPEED_DEADZONE_HWIDTH) - SPEED_DEADZONE_HWIDTH;
    }
  }
  analogWrite(PIN_MOTOR_A_PWM, (int)(255.*fabs(adjusted_speed)));
  analogWrite(PIN_MOTOR_B_PWM, (int)(255.*fabs(adjusted_speed)));
  if (adjusted_speed > 0){
    digitalWrite(PIN_MOTOR_A_DIR, HIGH);
    digitalWrite(PIN_MOTOR_B_DIR, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_A_DIR, LOW);
    digitalWrite(PIN_MOTOR_B_DIR, LOW);
  }
  data.drive_command = speed;
}

void doControl(){
  // If the robot is on its side, zero out control and reset
  // position integrator.
  if (fabs(data.zero_angle_setting - data.est_angle) >= 0.3){
    setDriveCommand(0.0, false); // quiet down
    data.est_pos = 0.0;
  } else {
    // Setpoint outer loop -- control back to original
    // position, and also create a bias against the
    // direction we're been driving in the recent past
    float pos_error = 0.1 * data.est_pos +
      1.0 * data.drive_command_lp;
    pos_error = fmin(fmax(pos_error, -0.1), 0.1);
    data.zero_angle_adjusted = data.zero_angle_setting + pos_error;

    // Angle tracking inner loop -- PD control
    float angle_error = data.zero_angle_adjusted - data.est_angle;
    float drive_command = angle_error * 2.0 
      - 0.20 * data.est_angle_vel;
    setDriveCommand(drive_command); 
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_MOTOR_A_PWM, OUTPUT);
  pinMode(PIN_MOTOR_B_PWM, OUTPUT);
  pinMode(PIN_MOTOR_A_DIR, OUTPUT);
  pinMode(PIN_MOTOR_B_DIR, OUTPUT);
  pinMode(PIN_ZERO_BUTTON, INPUT);

  // Uninitialized values
  data.x_rate_offset = -1.0;
  // Expected ~1.5V offset in imu
  data.y_acc_offset = 1.5 / 0.3;
  data.z_acc_offset = 1.5 / 0.3;
  data.zero_angle_setting = 0.22;
  data.zero_angle_adjusted = data.zero_angle_setting;

  last_update = get_current_time();
  last_publish_time = last_update - PUBLISH_PERIOD;
  start_time = last_update;

  setDriveCommand(0.0, false);
}

void loop() {
  double now = get_current_time();

  float dt = (float)(now - last_update);
  doStateEstimate(dt);
  doControl();

  last_update = now;

  if (now - last_publish_time > PUBLISH_PERIOD){
    send_IMU_struct();
    last_publish_time = now;
  }
}
