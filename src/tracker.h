#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

struct obstacle{
  int id;
  int pointNum;
  geometry_msgs::Twist path;
  geometry_msgs::Point lastPoints[3];
  nav_msgs::Odometry calculatedOdom;
  
  obstacle* next;
  
  void addNewPoint(geometry_msgs::Point);  
  obstacle();
};


class callback{
  public:
    callback(ros::NodeHandle nh);
    ~callback(){}
    
    obstacle* head;
    obstacle* tempHead;
    obstacle startObstacle;
    obstacle endObstacle;
    
    bool mapOk;
  private:
    nav_msgs::OccupancyGrid map;
    float robotViewAngle, robotMinRange, robotMaxRange;
    geometry_msgs::Pose robotPose;
    
    ros::Subscriber subOdom;
    ros::Subscriber subObstacle1_Odom;
    ros::Subscriber subObstacle2_Odom;
    ros::Subscriber subMap;
    
    bool lineCollisionCheck(geometry_msgs::Point, geometry_msgs::Point);
    bool collisionCheck(geometry_msgs::Point);    
    
    void mapCallback(const nav_msgs::OccupancyGrid&);
    void odomCallback(const nav_msgs::Odometry&);
    //saves robotPose. Just for checking if robot can see Obstacles
    void obstacle1_OdomCallback(const nav_msgs::Odometry&);
    void obstacle2_OdomCallback(const nav_msgs::Odometry&);
    //tracks robot_1. sends sampled position and if it's visible from robot_0 to trackingUpdate
};


nav_msgs::Odometry odomCalc(geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point, int);
//calculates trajectrory from three points. returns position, orientation and twist in odom

bool trackedObject(obstacle*, ros::Publisher*, int);
//checks all tracked objects and calculates trajectories if three points are available.
//publishes trajectories to /tracking

void trackingUpdate(int, bool, geometry_msgs::Point, obstacle**, obstacle**);
//takes newly sampled point and adds to object. 
//if sampling was unable to detect new position (e.g. not in view), object is deleted (for now).

float pointDistance(geometry_msgs::Point, geometry_msgs::Point);
//returns 2d distance of two points as float
