// #include <mbed.h>

// DigitalOut led1(PB_10);
// DigitalOut led2(PB_4);
// DigitalOut led3(PB_5);

// DigitalIn button1(PB_3);
// DigitalIn button2(PA_10);

// int main()
// {
//     while (1)
//     {
//         if (button1.read() == 0 || button2.read() == 0)
//         {
//             led1.write(1);
//             led2.write(0);
//             led3.write(1);
//         }
//         else
//         {
//             led1.write(0);
//             led2.write(1);
//             led3.write(0);
//         }
//     }
// }
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <mpu9250.h>
#include <sensor_msgs/Imu.h>
// MPU9250 imu(PB_9, PB_8);
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("uthai/sensor/imu", &imu_msg);
DigitalOut myled(LED1);

void messageCb(const std_msgs::Empty &toggle_msg)
{
    myled = !myled; // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

int main()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(imu_pub);
    imu_msg.header.frame_id = "base_stabilized";
    wait(1.0);
    Timer tim;
    tim.start();
    while (1)
    {

        // imu.updateData();
        if (tim.read_ms() > 100)
        {
            imu_msg.header.stamp = nh.now();
            // imu_msg.angular_velocity.x = imu.gyroData[0];
            // imu_msg.angular_velocity.y = imu.gyroData[1];
            // imu_msg.angular_velocity.z = imu.gyroData[2];
            // imu_msg.linear_acceleration.x = imu.accelData[0];
            // imu_msg.linear_acceleration.y = imu.accelData[1];
            // imu_msg.linear_acceleration.z = imu.accelData[2];
            // imu_msg.orientation.x = imu.q[0];
            // imu_msg.orientation.y = imu.q[1];
            // imu_msg.orientation.z = imu.q[2];
            // imu_msg.orientation.w = imu.q[3];
            imu_pub.publish(&imu_msg);
            tim.reset();
        }
        nh.spinOnce();
    }
}