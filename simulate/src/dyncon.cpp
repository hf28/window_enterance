#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "simulate/imtodyn.h"
#include <sstream>
using namespace std;

float vy, vz;
bool flag = false, enterance = false, go = false;
int init = 0;

void dataCallback(const simulate::imtodyn msg)
{
  cout<<"\n\n\n";
  flag = true;
  init++;
  vy = -(0.005*msg.y);
  vz = -(0.005*msg.z);
  enterance = msg.enterance;
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
  int count = 0, subCount = 0, t = 0 ;
  geometry_msgs::Twist gmsg;

  vey = gain*vy;
  vez = gain*vz;

  while (ros::ok())
  {
    cout<<"dyncon flag:\t"<<flag<<"\tentrance flag:  "<<enterance<<endl;


    if(vy>0.75) vy = 0.75;
    if(vz>0.75) vz = 0.75;
    if(vy<-0.75) vy = -0.75;
    if(vz<-0.75) vz = -0.75;
    cout<<"dyncon pd data\tvy:"<<vy<<"\tvz:\t"<<vz<<endl;

    if(init>=10) go = true;

    if(flag && go){
      gmsg.linear.x = 0.5;
      if(vy>0.75) gmsg.linear.y = vy;
      else gmsg.linear.y = 0;
      gmsg.linear.z = vz;
      gmsg.angular.x = 0;
      gmsg.angular.y = 0;
      gmsg.angular.z = 0.5*vy;
      flag = false;
      subCount++;
      t = 0;
    }
    else if(enterance && go){
      gmsg.linear.x = 0.5;
      gmsg.linear.y = 0;
      gmsg.linear.z = 0;
      gmsg.angular.x = 0;
      gmsg.angular.y = 0;
      gmsg.angular.z = 0;
      flag = false;
      subCount++;
      t = 0;
    }
    // else if(t<10 && !flag){
    //   ++t;
    //   gmsg.linear.x = 0;
    //   if(vy>0.75) gmsg.linear.y = -vy;
    //   else gmsg.linear.y = 0;
    //   gmsg.linear.z = -(0.5*vz);
    //   gmsg.angular.x = 0;
    //   gmsg.angular.y = 0;
    //   gmsg.angular.z = 0.2*(-vy);
    // }
    else {
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
