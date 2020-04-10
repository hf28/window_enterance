#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "simulate/imtodyn.h"
#include <sstream>
using namespace std;

float vy, vz;
bool flag = false;

void dataCallback(const simulate::imtodyn msg)
{
  flag = true;
  vy = -(0.005*msg.y);
  vz = -(0.005*msg.z);
  cout<<"dyncon subscribed data\tvy:"<<vy<<"\tvz:\t"<<vz<<endl;
}



int main(int argc, char **argv)
{
  float gain = 1, vey, vez;
  ros::init(argc, argv, "dyncon");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("visual_info", 1, dataCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("quadrotor_1/cmd_vel", 10);
  ros::Rate loop_rate(10); // Loop at 10Hz
  int count = 0;
  geometry_msgs::Twist gmsg;

  vey = gain*vy;
  vez = gain*vz;



  while (ros::ok())
  {
    cout<<"dyncon flag:\t"<<flag<<endl;

    if(flag){
      gmsg.linear.x = 0.5;
      gmsg.linear.y = vy;
      gmsg.linear.z = vz;
      gmsg.angular.x = 0;
      gmsg.angular.y = 0;
      gmsg.angular.z = 0.0;
      flag = false;
    }
    // else if(itsOK){
    //   gmsg.linear.x = 0.1;
    //   gmsg.linear.y = 0;
    //   gmsg.linear.z = 0;
    //   gmsg.angular.x = 0;
    //   gmsg.angular.y = 0;
    //   gmsg.angular.z = 0.0;
    //   flag = false;
    // }
    else{
      gmsg.linear.x = 0;
      gmsg.linear.y = 0.0;
      gmsg.linear.z = 0.0;
      gmsg.angular.x = 0;
      gmsg.angular.y = 0;
      gmsg.angular.z = 0.0;
    }
    cout<<"dyncon published data\tgmsg.linear.y:"<<gmsg.linear.y<<"\tgmsg.linear.z:\t"<<gmsg.linear.z<<endl;

    pub.publish(gmsg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
