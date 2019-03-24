/*
 * 
ServoControl.ino

This script adjusts throttle and steering servo servo positions.

If the transmitter's 'GEAR' toggle switch is in the '1' position, 
then the car is in AUTONOMOUS MODE and the steering and throttle 
servo values are being set by the ROS topics: "steering_servo_position"
and "throttle_servo_position".

If the transmitter's 'GEAR' toggle switch is in the '0' position, 
then the car is in MANUAL MODE and the steering and throttle 
servo values are being set by the position of the thumbsticks 
on the Devo7 Transmitter. (left-horizontal stick controls left 
and right steering, right-vertical stick controls forward and
reverse throttle.)

NOTES:
  Equipment:
    Arduino Mega 2560
    Devo7 Transmitter
    Devo AX701 Receiver
    R/C Car

  ARDUINO IDE:
    Tools --> Programmer --> Arduino as ISP

  ROSSERIAL_ARDUINO NOTES:
    Version: 0.7.7

  Troubleshooting:
    Issue1: In manual mode, throttle sometimes 'jumps' (rear tires spin)
            when the steering stick is moved too quickly.  

    Issue1-FIX: Make sure the steering servo and the throttle servo(esc) 
                are using seperate 5V signal power sources.

------------------------------------------------------------------------

To run:

  1. Turn on Devo7.
  2. Power on R/C car power switch.
  3. Switch "GEAR" to '0' (MANUAL MODE).
  4. Move Devo7 sticks to control steering and throttle. 
     (may need to unplug steering servo power and plug back
     in as the current 5V black battery shuts itself off
     when not active for several minutes.) 
     
  
  5. On laptop bash terminal, run: roscore
  6. On laptop bash terminal, run: rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
     (should see the following 4 lines of output in the terminal window if connected properly:

      [INFO] [1553385106.143624]: ROS Serial Python Node
      [INFO] [1553385106.160635]: Connecting to /dev/ttyACM0 at 57600 baud
      [INFO] [1553385110.112526]: Note: subscribe buffer size is 512 bytes
      [INFO] [1553385110.114268]: Setup subscriber on steering_servo_position [std_msgs/UInt16]
      [INFO] [1553385110.128020]: Setup subscriber on throttle_servo_position [std_msgs/UInt16]

  7. Switch "GEAR" to '1' (AUTONOMOUS MODE). (Built in LED on Arduino should be on.)
  8. On laptop bash terminal, run: rostopic pub steering_pwm_value std_msgs/UInt16 "data: 1900" --once
     (front wheels should turn right.)
  9. On laptop bash terminal, run: rostopic pub steering_pwm_value std_msgs/UInt16 "data: 1100" --once
     (front wheels should turn left.)
  10. On laptop bash terminal, run: rostopic pub steering_pwm_value std_msgs/UInt16 "data: 1500" --once
     (front wheels should turn straight.)

  NOTE: If steering ever does not work, unplug and plug back in the steering servo's 5V power supply.
*/


//#include <ArduinoHardware.h>
#include <Servo.h>
#include "ros.h"
#include <std_msgs/UInt16.h>


/* ------------------------------------- THROTTLE ------------------------------------- */ 

// throttle variables
Servo throttle;  // create servo object to control the throttle servo
const int throttle_pin_in  = 2; // yellow wire
const int throttle_pin_out = 4; // blue wire
volatile int throttle_pwm_value_manual = 1500;
volatile int throttle_pwm_value_autonomous = 1500;
volatile int throttle_prev_time = 0;


// throttle - detect rising edge
void throttle_rising() {
  attachInterrupt(digitalPinToInterrupt(throttle_pin_in), throttle_falling, FALLING);
  throttle_prev_time = micros();
}
 
// throttle - detect falling edge
void throttle_falling() {
  attachInterrupt(digitalPinToInterrupt(throttle_pin_in), throttle_rising, RISING);
  throttle_pwm_value_manual = micros()-throttle_prev_time;
}


// throttle callback function
void throttle_cb( const std_msgs::UInt16& msg){
  if(msg.data >= 1100 && msg.data <= 1900)
  {
    throttle_pwm_value_autonomous = msg.data;
  }
}

// throttle ROS subscriber and node handle
ros::Subscriber<std_msgs::UInt16> throttle_sub("throttle_servo_position", &throttle_cb );
//ros::NodeHandle  throttle_nh;

/* ------------------------------------------------------------------------------------ */ 






/* ------------------------------------- STEERING ------------------------------------- */ 

// steering variables
Servo steering;  // create servo object to control the steering servo
const int steering_pin_in  = 3; // purple wire
const int steering_pin_out = 5; // green wire
volatile int steering_pwm_value_manual = 1500;
volatile int steering_pwm_value_autonomous = 1500;
volatile int steering_prev_time = 0;

// steering functions
void sweep_steering()
{
  steering.writeMicroseconds(1800); // steer max left
  delay(1000);
  steering.writeMicroseconds(1200); // steer max right
  delay(1000);
  steering.writeMicroseconds(1500); // steer straight
  delay(1000);
}

// steering - detect rising edge
void steering_rising() {
  attachInterrupt(digitalPinToInterrupt(steering_pin_in), steering_falling, FALLING);
  steering_prev_time = micros();
}
 
// steering - detect falling edge
void steering_falling() {
  attachInterrupt(digitalPinToInterrupt(steering_pin_in), steering_rising, RISING);
  steering_pwm_value_manual = micros()-steering_prev_time;
}

// steering callback function
void steering_cb( const std_msgs::UInt16& msg){
  if(msg.data >= 1100 && msg.data <= 1900)
  {
    steering_pwm_value_autonomous = msg.data;
  }
}

// steering ROS subscriber and node handle
ros::Subscriber<std_msgs::UInt16> steering_sub("steering_servo_position", &steering_cb );
//ros::NodeHandle  steering_nh;

/* ------------------------------------------------------------------------------------ */ 


ros::NodeHandle nh;




/* -------------------------------------- GEAR --------------------------------------- */ 

// gear variables
Servo gear;  // create servo object to control the gear servo
const int gear_pin_in  = 18; // brown wire
const int gear_pin_out = 6; // ??? wire
volatile int gear_pwm_value_manual = 1100; // start in 'manual mode' where Devo7 sticks have control
volatile int gear_pwm_value_autonomous = 1100; // start in 'manual mode' where Devo7 sticks have control
volatile int gear_prev_time = 0;
bool autonomous_mode = false;

// gear - detect rising edge
void gear_rising() {
  attachInterrupt(digitalPinToInterrupt(gear_pin_in), gear_falling, FALLING);
  gear_prev_time = micros();
}
 
// gear - detect falling edge
void gear_falling() {
  attachInterrupt(digitalPinToInterrupt(gear_pin_in), gear_rising, RISING);
  gear_pwm_value_manual = micros()-gear_prev_time;

  if(gear_pwm_value_manual > 1000 && gear_pwm_value_manual < 1500)
  {
    // turn autonomous_mode OFF
    autonomous_mode = false;
  }
  else if(gear_pwm_value_manual > 1500 && gear_pwm_value_manual < 2000)
  {
    // turn autonomous_mode ON
    autonomous_mode = true;
  }
}



/* ----------------------------------------------------------------------------------- */ 


// setup
void setup() {
  
  // when pin D2 (throttle) goes high, call the rising function
  attachInterrupt(digitalPinToInterrupt(throttle_pin_in), throttle_rising, RISING);
  // when pin D3 (steering) goes high, call the rising function
  attachInterrupt(digitalPinToInterrupt(steering_pin_in), steering_rising, RISING);
  // when pin D18 (gear) goes high, call the rising function
  attachInterrupt(digitalPinToInterrupt(gear_pin_in), gear_rising, RISING);
  
  // attach servos to pins
  throttle.attach(throttle_pin_out);
  steering.attach(steering_pin_out);
  //gear.attach(gear_pin_out);

//  // init nodes and subscribe to topics
//  steering_nh.initNode();
//  steering_nh.subscribe(steering_sub);
//
//  throttle_nh.initNode();
//  throttle_nh.subscribe(throttle_sub);

  nh.initNode();
  nh.subscribe(steering_sub);
  nh.subscribe(throttle_sub);

  // LED ON == autonomous mode. LED OFF == manual mode
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  sweep_steering();

}


// loop
void loop()
{
  
  nh.spinOnce();
  
  // write servo values
  if(autonomous_mode)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    throttle.writeMicroseconds(throttle_pwm_value_autonomous);
    steering.writeMicroseconds(steering_pwm_value_autonomous);
 
  }

  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    throttle.writeMicroseconds(throttle_pwm_value_manual);
    steering.writeMicroseconds(steering_pwm_value_manual);
   
  }

}
