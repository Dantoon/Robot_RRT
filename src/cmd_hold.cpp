#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

class pubSub
{
  public:
    pubSub(ros::NodeHandle);
    ~pubSub(){}
    
    ros::Publisher pubCmd;
    geometry_msgs::Twist cmd;
  private:
    ros::Subscriber subCmd;
    
    void cmdCallback(const geometry_msgs::Twist&);
};

//~~~~~~~~~~~~~~ M A I N ~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_hold");
  ros::NodeHandle nh;
  pubSub pubSub(nh);
  ros::Rate spinRate(100);
  
  while(ros::ok()){
    ros::spinOnce();
    pubSub.pubCmd.publish(pubSub.cmd);
    //spinRate.sleep();
  }
  
  return 0;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

pubSub::pubSub(ros::NodeHandle nh)
{
  cmd.linear.x = 0.0f;
	cmd.linear.y = 0.0f;
	cmd.linear.z = 0.0f;
	cmd.angular.x = 0.0f;
	cmd.angular.y = 0.0f;
	cmd.angular.z = 0.0f;
	
	subCmd = nh.subscribe("/rrt_cmd", 10, &pubSub::cmdCallback, this);
	pubCmd = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1, false);
}

void pubSub::cmdCallback(const geometry_msgs::Twist& msg)
{
  cmd = msg;
}
