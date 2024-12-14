#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "CytronMotorDriver.h"


// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Array to store current positions of each servo
float cur_pos[6] = {0, 0, 0, 0, 0, 0};

// Servo objects
Servo servos[6];

// Publisher and message for joint states
ros::Publisher joint_pub("/joint_states", new rospy_tutorials::Floats());
rospy_tutorials::Floats joint_msg;

void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  lastCmdVelReceived = (millis()/1000);
   
  // Calculate the PWM value given the desired velocity
  double basePWM = K_P * fabs(cmdVel.linear.x) + b;

  // Read a value from 0 to 1
  float value = cmdVel.linear.x / 1023.0;  // Assuming you're reading from analog pin A0

  // Map the value to 0 to 180
  int mappedValue = value * 180;

 if (cmdVel.linear.x==0){
      servos[1].write(current1);
      servos[2].write(current2);
      servos[3].write(current3);
      servos[4].write(current4);
      servos[5].write(current5);
 }
  else{
     if (cmdVel.linear.x==0){  
      current1 = value * 180;
      servos[1].write(current1);
       
  }
}
// Function to rotate a servo smoothly
void rotate_servo(int servo_index, int new_pos) {
  if (servo_index < 0 || servo_index >= 6) return; // Invalid servo index

  int cur = cur_pos[servo_index];
  if (new_pos < 0) new_pos = 0;   // Lower limit
  if (new_pos > 180) new_pos = 180; // Upper limit

  // Smooth movement
  if (new_pos > cur) {
    for (int pos = cur; pos <= new_pos; ++pos) {
      servos[servo_index].write(pos);
      delay(60);  // Smooth delay
    }
  } else if (new_pos < cur) {
    for (int pos = cur; pos >= new_pos; --pos) {
      servos[servo_index].write(pos);
      delay(60);  // Smooth delay
    }
  }
  cur_pos[servo_index] = new_pos; // Update the current position
}

// Callback for /joints_to_aurdino topic
void servo_cb(const rospy_tutorials::Floats& cmd_msg) {
  nh.loginfo("Command Received");

  for (int i = 0; i < 6; ++i) {
    if (i < cmd_msg.data_length) {
      rotate_servo(i, cmd_msg.data[i]);
    }
  }
}

// Service callback to return joint states
void joint_state_cb(const robotic_armv5::Floats_array::Request& req, robotic_armv5::Floats_array::Response& res) {
  res.res_length = 6;
  for (int i = 0; i < 6; ++i) {
    res.res[i] = cur_pos[i]; // Return current joint positions
  }
}

// Subscriber and Service definitions
ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", servo_cb);
ros::ServiceServer<robotic_armv5::Floats_array::Request, robotic_armv5::Floats_array::Response> joint_service("/get_joint_commands", &joint_state_cb);
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );

void setup() {
  nh.initNode();
  nh.subscribe(sub);
    nh.subscribe(subCmdVel);
  nh.advertise(joint_pub);
  nh.advertiseService(joint_service);

  // Attach servos to pins
  servos[0].attach(6); 
  servos[1].attach(9); 
  servos[2].attach(11); 
  servos[3].attach(3); 
  servos[4].attach(5); 
  servos[5].attach(10); 

  // Initialize servo positions
  for (int i = 0; i < 6; ++i) {
    servos[i].write(cur_pos[i]);
  }

  // Initialize joint_msg
  joint_msg.data_length = 6;
  joint_msg.data = cur_pos;
}

void loop() {
  nh.spinOnce();

  // Publish joint states
  joint_msg.data_length = 6;
  joint_msg.data = cur_pos;
  joint_pub.publish(&joint_msg);

  delay(80); // Small delay to avoid flooding
}
