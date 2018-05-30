#include <ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

uint16_t sensors[3];
geometry_msgs::Vector3 force;
ros::Publisher gcs("uthai/l_gcs", &force);

void setup() {
  nh.initNode();
  nh.advertise(gcs);
}

void loop() {
  sensors[0] = map(analogRead(0), 0, 1023, 0, 1023);
  sensors[1] = map(analogRead(1), 0, 1023, 0, 1023);
  sensors[2] = map(analogRead(2), 0, 1023, 0, 1023);
  force.x = sensors[0];
  force.y = sensors[1];
  force.z = sensors[2];
  gcs.publish(&force);
  nh.spinOnce();
  delay(100);
}
