#include "ros/ros.h"
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "dong_core/sensor.h" 
#include "time.h"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define LINEAR                           0
#define ANGULAR                          1
#define WHEEL_NUM                        2

#define WHEEL_RADIUS                     0.0625 //휠 반지름 m단위        
#define WHEEL_SEPARATION                 0.160
#define TURNING_RADIUS                   0.080 

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 
#define LEFT                             0
#define RIGHT                            1
#define TURNING_RADIUS                   0.080 
#define TICK2RAD                         0.001533981

float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
unsigned long prev_update_time;

char odom_header_frame_id[30];
char odom_child_frame_id[30];
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
float yaw_angle = 0;
float odom_pose[3];
double odom_vel[3];
#define FIRMWARE_VER "1.2.6"
unsigned char flag = 0;
unsigned char cmd_flag = 0;

bool isChecked;
ros::Time rosNow(void);
clock_t t; 
clock_t end;
static uint32_t tTime[10];
float constrain(float input, float low, float high);
void updateGoalVelocity(void);
void publishDriveInformation(void);
void publishSensorStateMsg(void);
bool calcOdometry(double diff_time);
void updateOdometry(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateJointStates(void);
void updateMotorInfo(double left_tick, double right_tick);
void updateTFPrefix(bool isConnected);
void updateVariable(bool isConnected);
void initOdom(void);

nav_msgs::Odometry odom;

geometry_msgs::TransformStamped odom_tf;
sensor_msgs::JointState joint_states;

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = clock();
  cmd_flag = 1;

}

void msgCallback(const dong_core::sensor::ConstPtr& msg)
{
  updateMotorInfo(msg->left_encoder,msg->right_encoder);
  yaw_angle = msg->yaw_angle;
  unsigned long time_now = clock();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = ros::Time::now();
  calcOdometry((double)(step_time * 0.001));
  updateOdometry(); 
  odom.header.stamp = stamp_now;

  flag = 1;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "core_pub");
  ros::NodeHandle nh;
  

  geometry_msgs::Twist twist;
  tf::TransformBroadcaster tf_broadcaster;
  turtlebot3_msgs::VersionInfo version_info_msg;
  turtlebot3_msgs::SensorState sensor_state_msg;
  dong_core::sensor sensor_msg;
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 100, commandVelocityCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry >("odom", 100);
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_serial", 100);
  ros::Publisher joint_states_pub = nh.advertise<turtlebot3_msgs::SensorState>("joint_states", 100);
  ros::Subscriber sensor_sub = nh.subscribe("sensor_encoder", 5, msgCallback);
  ros::Publisher sensor_pub = nh.advertise<dong_core::sensor >("sensor_encoder_1", 5);
  ros::Publisher version_info_pub = nh.advertise<turtlebot3_msgs::VersionInfo>("firmware_version", 100);

  void publishImuMsg(void);

  char get_prefix[10];
  std::string get_tf_prefix = get_prefix;

  char imu_frame_id[30];
  char mag_frame_id[30];

  char joint_state_header_frame_id[30];
  tf::Transform transform;
  std::string turtle_name;
  turtle_name = argv[1];
  tf_broadcaster.sendTransform(odom_tf);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  while(ros::ok()){
  ros::spinOnce();
  t = clock();

  isChecked = false;
    
    if (isChecked == false)
    {
      nh.getParam("tf_prefix",get_tf_prefix);

      if (!strcmp(get_tf_prefix.c_str(), ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix.c_str());
        strcpy(odom_child_frame_id, get_tf_prefix.c_str());

        strcpy(joint_state_header_frame_id, get_tf_prefix.c_str());

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

      }
      isChecked = true;
    }

    if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY)) //33ms 마다 출력 되는거 확인
    {
      updateGoalVelocity();
      if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) 
      {
        if(cmd_flag == 1)
        {
          cmd_vel_pub.publish(twist);
          cmd_flag = 0;
        } 
      } 
      else {
        if(cmd_flag == 1)
        {
          twist.linear.x = goal_velocity[LINEAR];
          twist.angular.z = goal_velocity[ANGULAR];
          ROS_INFO("%f,%f",twist.linear.x ,twist.angular.z);
          cmd_vel_pub.publish(twist); 
          cmd_flag = 0;
        }
      }
      tTime[0] = t;
    }

    if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
    {
      sensor_msg.quest = true;
      sensor_pub.publish(sensor_msg);
   
      if(flag == 1)
      {
        odom_pub.publish(odom);

        updateTF(odom_tf);
        ros::Time stamp_now = ros::Time::now();
        odom_tf.header.stamp = stamp_now;
        tf_broadcaster.sendTransform(odom_tf);
        joint_states.header.stamp = stamp_now;
        joint_states_pub.publish(joint_states);
      
      }
      tTime[2] = t;
    }

    if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
    {
      tTime[3] = t;
    }

    if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
    {
      version_info_msg.hardware = "0.0.0";
      version_info_msg.software = "0.0.0";
      version_info_msg.firmware = FIRMWARE_VER;

    version_info_pub.publish(version_info_msg);
      tTime[4] = t;
    }
  }
  return 0;
}

void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}

float constrain(float input, float low, float high)
{
  if (input < low)
  {  input = low;}
  else if(input > high)
  {  input = high;}
  else
  { input = input;}

  return input;
}

void updateMotorInfo(double left_tick, double right_tick)
{
  double current_tick = 0;
  static double last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

last_tick[LEFT]      = left_tick;
last_tick[RIGHT]      = right_tick;
//  current_tick = left_tick;

//  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
//  last_tick[LEFT]      = current_tick;

//  current_tick = right_tick;

//  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
//  last_tick[RIGHT]      = current_tick;
}

bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = (double)last_diff_tick[LEFT];
  wheel_r = (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  theta = yaw_angle;
  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = theta - last_theta; 

  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

void updateOdometry(void)
{
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_pose[2]);

  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;
  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];
}

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}
