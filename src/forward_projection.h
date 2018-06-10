#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"

class callback{
  public:
    callback(ros::NodeHandle);
    ~callback(){}
    
    nav_msgs::Odometry currentOdom;
    nav_msgs::OccupancyGrid map;
    std_msgs::Int16 cmdNum;
    geometry_msgs::Twist* cmdArr;
    geometry_msgs::Pose startPose;
    geometry_msgs::Point goal;
    
    bool mapOk;
    bool goalOk;
    bool cmdOk;
    bool cmdNumOk;
    bool startOk;
    int cmdCallbackCntr;
    int cmdCntr;
    int cmdStart;
    
    bool collisionCheck(geometry_msgs::Point);
    geometry_msgs::Pose projection(geometry_msgs::Pose, geometry_msgs::Twist);
    void replan();
    
  private:
    ros::Subscriber subOdom;
    ros::Subscriber subMap;
    ros::Subscriber subGoal;
    ros::Subscriber subCmdNum;
	  ros::Subscriber subCmd;
	  ros::Subscriber subStartPose;
	  ros::Subscriber subReplan;
	  ros::Subscriber subObstacle;
    
    void odomCallback(const nav_msgs::Odometry&);
    void mapCallback(const nav_msgs::OccupancyGrid&);
    void goalCallback(const geometry_msgs::Point&);
    void cmdCallback(const geometry_msgs::Twist&);
    void cmdNumCallback(const std_msgs::Int16&);
    void startPoseCallback(const geometry_msgs::Twist&);
    void replanCallback(const std_msgs::Bool&);
    void obstacleCallback(const nav_msgs::Odometry&);
    
    ros::Publisher pubReplan;
};


geometry_msgs::Pose projection(geometry_msgs::Pose, geometry_msgs::Twist);

float pointDistance(geometry_msgs::Point, geometry_msgs::Point);

visualization_msgs::Marker markerPoint(geometry_msgs::Pose, int);

visualization_msgs::Marker clearMarker();

