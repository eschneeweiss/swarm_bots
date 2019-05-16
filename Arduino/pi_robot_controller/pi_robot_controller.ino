/* This example drives each motor connected to the A-Star
32U4 Robot Controller forward, then backward.  The yellow
user LED is on when a motor is set to a positive speed and
off when a motor is set to a negative speed. */

#include <AStar32U4.h>
#include <PololuRPiSlave.h>

struct Data
{
  // format: https://docs.python.org/2/library/struct.html
  
  // address 0, format "BBB"
  bool yellow, green, red;
  // address 3, format "BBB"
  bool buttonA, buttonB, buttonC;

  // address 6, format "H"
  uint16_t batteryMillivolts;

  // address 8, format "B15s"
  bool playNotes;
  char notes[14];

  // address 23, format "fff"
  float odom_x, odom_y, odom_theta;

  // address 35, format "ff"
  float vel_tran, vel_rot;
};

// library variables:
PololuRPiSlave<struct Data,5> slave;
AStar32U4Motors motors;
PololuBuzzer buzzer;
AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;

// robot constants
const float wheel_diameter = 32; // 32mm
const float wheel_radius = wheel_diameter/2.0;
const float wheel_circumference = wheel_diameter*PI;
const float wheel_distance = 95; // 95mm

const int encoder_ticks_per_rev = 360;
const float distance_per_tick = wheel_circumference/float(encoder_ticks_per_rev);

const int max_rpm = 120;
const float max_speed = float(max_rpm)*(1.0/60.0)*wheel_circumference; // 201.06
const float max_rot_speed = (2*max_speed)/wheel_distance; // 4.23

// variables for motor control
int encoder1A = 0;
int encoder1B = 4;
long enc1 = 0;
float distance1 = 0;
float prev_distance1 = 0;

int encoder2A = 1;
int encoder2B = 5;
long enc2 = 0;
float distance2 = 0;
float prev_distance2 = 0;

// struct for PID controller
struct motor_PID
{
  float desired_velocity;
  int set_speed = 0;
  float velocity = 0;
  float prev_velocity = 0;
  float acceleration = 0;
  // weights
  float p_weight = 1.0;
  public:
  void update_PID(float delta_time, float delta_distance)
  {
    // update velocity and acceleration
    velocity = delta_distance/delta_time;

    float delta_velocity = velocity - prev_velocity;
    prev_velocity = velocity;

    acceleration = delta_velocity/delta_time;

    // calculate PID error
    float p_val = desired_velocity - velocity;
  
    float error = p_val*p_weight;

    // update set speed
    set_speed += error;
    if (set_speed > 400)
    {
      set_speed = 400;
    }
    else if (set_speed < -400)
    {
      set_speed = -400;
    }
  }
  void set_PID(float desired_velocity)
  {
    this->desired_velocity = desired_velocity;
  }
};

motor_PID motor1_PID; 
motor_PID motor2_PID;

// timer variables
unsigned long currentMillis = millis();
unsigned long previousMillis = currentMillis;

// odometry: x, y, theta
float odom_x, odom_y, odom_theta;

void update_odometry(float left_distance, float right_distance)
{
  // update odometry
  //odometry[2] = right_distance - left_distance;
  
}

// rpi velocity: translation, rotation
float vel_tran, vel_rot;
// Robot left wheel velocity, and right wheel velocity
float vel_left, vel_right;

void update_velocities(float vel_tran, float vel_rot)
{
  // cap speed
  if (vel_tran > max_speed)
  {
    vel_tran = max_speed;
  }
  else if (vel_tran < -max_speed)
  {
    vel_tran = -max_speed;
  }
  
//  vel_left = -(vel_tran - vel_rot*wheel_distance/2.0);
//  vel_right = (vel_tran + vel_rot*wheel_distance/2.0);

  vel_left = vel_tran - vel_rot*wheel_distance/2.0;
  vel_right = vel_tran + vel_rot*wheel_distance/2.0;

  if (vel_left > max_speed)
  {
    vel_right = vel_right - (vel_left - max_speed);
    vel_left = max_speed;
  }
  else if (vel_right > max_speed)
  {
    vel_left = vel_left - (vel_right - max_speed);
    vel_right = max_speed;
  }
  else if (vel_left < -max_speed)
  {
    vel_right = vel_right - (vel_left + max_speed);
    vel_left = max_speed;
  }
  else if (vel_right > max_speed)
  {
    vel_left = vel_left - (vel_right + max_speed);
    vel_right = max_speed;
  }

  // flip sign due to motor orientation
  vel_left = -vel_left;
}



// encoder
void encoder1A_trigger()
{
  if(digitalRead(encoder1B) == 1)
  {
    enc1++;
  }
  else
  {
    enc1--;
  }
}

void encoder2A_trigger()
{
  if(digitalRead(encoder2B) == 1)
  {
    enc2++;
  }
  else
  {
    enc2--;
  }
}

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");

  // setup encoders for pid controller
  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_trigger, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2A), encoder2A_trigger, RISING);
}

//max speed is 400
void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write button values
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  // write battery voltage
  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivoltsSV();

  // write odometry
  slave.buffer.odom_x = odom_x;
  slave.buffer.odom_y = odom_y;
  slave.buffer.odom_theta = odom_theta;

  // READING the buffer is allowed before or after finalizeWrites().
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);

  // Playing music involves both reading and writing, since we only
  // want to do it once.
  static bool startedPlaying = false;
  
  if(slave.buffer.playNotes && !startedPlaying)
  {
    buzzer.play(slave.buffer.notes);
    startedPlaying = true;
  }
  else if (startedPlaying && !buzzer.isPlaying())
  {
    slave.buffer.playNotes = false;
    startedPlaying = false;
  }

  // read velocity
  vel_tran = slave.buffer.vel_tran;
  vel_rot = slave.buffer.vel_rot;
  

  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();


  // Diagnostic
  Serial.print("Max speed: ");
  Serial.print(max_speed);
  Serial.print(", Max rotation speed: ");
  Serial.println(max_rot_speed);

  
//  Serial.print("d1: ");
//  Serial.print(enc1);
//  Serial.print(", d2: ");
//  Serial.println(enc2);
//  Serial.println(max_speed);

  currentMillis = millis();
  float delta_time = (currentMillis - previousMillis)/1000.0; //convert to seconds
  previousMillis = currentMillis;

  distance1 = float(enc1)*distance_per_tick; // convert to mm
  float delta_distance1 = distance1-prev_distance1;
  prev_distance1 = distance1;

  distance2 = float(enc2)*distance_per_tick; // convert to mm
  float delta_distance2 = distance2-prev_distance2;
  prev_distance2 = distance2;
  
  motor1_PID.update_PID(delta_time, delta_distance1);
  motor2_PID.update_PID(delta_time, delta_distance2);
  update_velocities(vel_tran, vel_rot);
  //update_velocities(0, 4);

  motor1_PID.set_PID(vel_right);
  motor2_PID.set_PID(vel_left);
  motors.setM1Speed(motor1_PID.set_speed);
  motors.setM2Speed(motor2_PID.set_speed);
  delay(100);
}
