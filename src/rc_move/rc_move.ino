/* Include necessary libraries for ROS and servo */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

/* initialize default values */

Servo myservo;
ros::NodeHandle nh;

int steer;

/* callback function used when message is receieved*/
void messageCb( const std_msgs::Int16& cmd_vel) {
  steer = cmd_vel.data;   //set global variable to received value
  myservo.write(steer);
}

std_msgs::Int16 pub_steer;
/* Subscribe to /cmd_vel , publish to /steer_return */
ros::Subscriber<std_msgs::Int16> sub("/cmd_vel", &messageCb );
ros::Publisher pub("/steer_return", &pub_steer);

void setup() { 
/* Initialize all pin modes, init ROS */ 
  myservo.attach(9); 
  nh.initNode(); 
  nh.subscribe(sub); 
  nh.advertise(pub); 
  steer = 110;
  myservo.write(steer); 
} 

void loop() { 
  /* ROS spinonce */
  nh.spinOnce();
  /* step motor control */
  //myservo.write(steer);
  pub_steer.data = steer;
  pub.publish(&pub_steer);


  delay(100);
}

