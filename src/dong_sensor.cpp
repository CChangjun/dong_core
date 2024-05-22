#include "ros/ros.h"                                            
#include <stdio.h>
#include <unistd.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "dong_core/sensor.h" 
#include <geometry_msgs/Twist.h>

serial::Serial ser;


char BufferPacket[] = {0,};
float left_encoder = 0;
float right_encoder = 0;
float yaw_angle = 0;
unsigned char flag = 0;

dong_core::sensor sensor_msg;

uint8_t testStr_1[] = {0xab};

void checkCallBACK(const dong_core::sensor::ConstPtr& msg)
{
  sensor_msg.quest = msg->quest;

  if(sensor_msg.quest==true)
  {
    //ser.write(testStr_1,1);
    flag = 1;
    sensor_msg.quest = false;
  }
}

void ser_msgCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  uint8_t linear_x_8bit =  (uint8_t)(msg->linear.x);
  uint8_t angular_z_8bit = (uint8_t)(msg->angular.z);

  snprintf(BufferPacket, sizeof(BufferPacket), "%.1f,%.1f\n", msg->linear.x, msg->angular.z);
  //### initializing argument :Serial::write(const uint8_t*, size_t)’
  ser.write(BufferPacket);
  //ROS_INFO("send fin %d,%d", BufferPacket[0],BufferPacket[1]);
  ROS_INFO("--------------------------------");
}


int main(int argc, char **argv)                         
{
  ros::init(argc, argv, "dong_sensor"); 
  ros::NodeHandle nh;                                   
  ros::Publisher sensor_pub = nh.advertise<dong_core::sensor>("sensor_encoder", 10);
  ros::Subscriber dong_ros_sub = nh.subscribe("cmd_vel_serial", 100, ser_msgCallback);
  ros::Subscriber sensor_sub = nh.subscribe("sensor_encoder_1", 5, checkCallBACK);

  sensor_msg.quest = false;
  
  try
  {
      ser.setPort("/dev/ttyFT232");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
  }

  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if(ser.isOpen()){
      ROS_INFO_STREAM("Serial Port initialized");
  }else{
     return -1;
  }
  ros::Duration(1).sleep();
  char msg[30];

  ros::Rate loop_rate(5);

  while(ros::ok()){
      ros::spinOnce();

      if(ser.available()){
      char *dend;

      std_msgs::String result;

      result.data = ser.read(ser.available());

      strcpy(msg,result.data.c_str()); //string -> char로 변환 

    if(flag == 1)
    {
        left_encoder = strtof(msg,&dend);
        right_encoder = strtof(dend+1,&dend);
        yaw_angle = strtof(dend+1,NULL);
 
//        ROS_INFO("%f,%f,%f",sensor_msg.left_encoder,sensor_msg.right_encoder,sensor_msg.yaw_angle);
//        sensor_msg.left_encoder = left_encoder*0.0325; 
//        sensor_msg.right_encoder = right_encoder*0.0325; 32.5ms로 변환
        sensor_msg.left_encoder = left_encoder; 
        sensor_msg.right_encoder = right_encoder; //100ms로 변환       
        sensor_msg.yaw_angle = yaw_angle;   
        ROS_INFO("recive : left_encoder: %f, right_encoder : %f, yaw_angle : %f",left_encoder,right_encoder,yaw_angle);
        sensor_pub.publish(sensor_msg);

        flag = 0;
     }

      }
    loop_rate.sleep();
    }
}
