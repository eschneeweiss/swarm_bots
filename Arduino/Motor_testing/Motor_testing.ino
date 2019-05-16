/* This example drives each motor connected to the A-Star
32U4 Robot Controller forward, then backward.  The yellow
user LED is on when a motor is set to a positive speed and
off when a motor is set to a negative speed. */

#include <AStar32U4.h>

AStar32U4Motors motors;

float wheel_diameter = 32; // 32mm
float wheel_circumference = wheel_diameter*PI;
float wheel_distance = 95; // 95mm

int encoder_ticks_per_rev = 360;
float distance_per_tick = wheel_circumference/float(encoder_ticks_per_rev);

int max_rpm = 120;
float max_speed = float(max_rpm)*(1/60)*wheel_circumference;

// variables for motor control
int encoder1A = 0;
int encoder1B = 4;
int set_speed1 = 0;
long enc1 = 0;
float distance1 = 0;
float prev_distance1 = 0;
float velocity1 = 0;
float prev_velocity1 = 0;
float acceleration1 = 0;

int encoder2A = 1;
int encoder2B = 5;
int set_speed2 = 0;
long enc2 = 0;
float distance2 = 0;
float prev_distance2 = 0;
float velocity2 = 0;
float prev_velocity2 = 0;
float acceleration2 = 0;

// timer variables
unsigned long currentMillis = millis();
unsigned long previousMillis = currentMillis;

void update_vel_accel()
{
  currentMillis = millis();
  distance1 = float(enc1)*distance_per_tick;
  distance2 = float(enc2)*distance_per_tick;
  
  float delta_time = (currentMillis - previousMillis)/1000.0; // convert to seconds
  previousMillis = currentMillis;
  
  float delta_distance1 = distance1 - prev_distance1;
  prev_distance1 = distance1;
  float delta_distance2 = distance2 - prev_distance2;
  prev_distance2 = distance2;
  
  velocity1 = delta_distance1/delta_time;
  velocity2 = delta_distance2/delta_time;
  
  float delta_velocity1 = velocity1 - prev_velocity1;
  prev_velocity1 = velocity1;
  float delta_velocity2 = velocity2 - prev_velocity2;
  prev_velocity2 = velocity2;
  
  acceleration1 = delta_velocity1/delta_time;
  acceleration2 = delta_velocity2/delta_time;
}

void velocity_control(float desired_velocity)
{
  float p_val = desired_velocity - velocity1;
  float p_weight = 1.0;

  float error = p_val*p_weight;

  set_speed1 += error;
  if (set_speed1 > 400)
  {
    set_speed1 = 400;
  }
  else if (set_speed1 < -400)
  {
    set_speed1 = -400;
  }

  motors.setM1Speed(set_speed1);
  
}

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
  // Uncomment to flip a motor's direction:
  //motors.flipM1(true);
  //motors.flipM2(true);
  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_trigger, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_trigger, RISING);
  //motors.setM1Speed(200);
}

//max speed is 400
void loop()
{
//  Serial.print("d1: ");
//  Serial.print(enc1);
//  Serial.print(", d2: ");
//  Serial.println(enc2);

  Serial.print("dist: ");
  Serial.print(distance1);
  Serial.print(", vel: ");
  Serial.print(velocity1);
  Serial.print(", accel: ");
  Serial.println(acceleration1);


  delay(100);

  update_vel_accel();
  velocity_control(50);
  
//  ledYellow(0);
//  delay(100);

//  ledGreen(1);
//  motors.setM1Speed(400);
//  delay(1000);
//  ledGreen(0);
//  ledRed(1);
//  motors.setM1Speed(0);
//  delay(1000);
//  ledRed(0);
}
