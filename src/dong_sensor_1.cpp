#include "ros/ros.h"                                            
#include <stdio.h>
#include <unistd.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "dong_core/sensor.h" 


serial::Serial ser;


char BufferPacket[] = {0,};
int left_encoder = 0;
int right_encoder = 0;
unsigned char flag = 0;

dong_core::sensor sensor_msg;

uint8_t testStr_1[] = {0xab};

void msgCallback(const dong_core::sensor::ConstPtr& msg)
{
  sensor_msg.quest = msg->quest;

  if(sensor_msg.quest==true)
  {
    ser.write(testStr_1,1);
    ROS_INFO_STREAM("ready");
    flag = 1;
    sensor_msg.quest = false;
  }
}

int main(int argc, char **argv)                         
{
  ros::init(argc, argv, "dong_sensor"); 
  ros::NodeHandle nh;                                   
  ros::Publisher sensor_pub = nh.advertise<dong_core::sensor>("sensor_encoder", 10);

  ros::Subscriber sensor_sub = nh.subscribe("sensor_encoder_1", 5, msgCallback);

  sensor_msg.quest = false;
  
  try
  {
      ser.setPort("/dev/ttyUSB0");
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

  ros::Rate loop_rate(10);

  while(ros::ok()){
        ros::spinOnce();

//        ser.write(testStr_1,2);
        
        if(ser.available()){
        char *dend;
       // ROS_INFO_STREAM("\nReading from serial port");
        std_msgs::String result;

        result.data = ser.read(ser.available());

        strcpy(msg,result.data.c_str()); //string -> char로 변환 

//        ROS_INFO_STREAM("Read: " << result.data);

        if(flag == 1)
        {
            left_encoder = strtof(msg,&dend);
            right_encoder = strtof(dend+1,NULL);
 //           ROS_INFO_STREAM("Read: " << result.data);
 //           ROS_INFO_STREAM("ok");
 //           ROS_INFO("%f,%f",sensor_msg.left_encoder,sensor_msg.right_encoder);
            sensor_msg.left_encoder = left_encoder*0.0325;
            sensor_msg.right_encoder = right_encoder*0.0325;           
            sensor_pub.publish(sensor_msg);
            flag = 0;
        }
//        ser.write(result.data);

        }
    loop_rate.sleep();
    }
}
