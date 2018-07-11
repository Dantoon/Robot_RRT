#include <tf/transform_datatypes.h>
#include "forward_projection.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "forward_projection");
  ros::NodeHandle nh;
  callback callback(nh);
  float tolerance = 2.0f;
  
  callback.pubMarker.publish(clearMarker());
  visualization_msgs::Marker nodeMarker;
  
  while(ros::ok())
  {
    ros::spinOnce();
    
    
    while(callback.cmdOk && callback.mapOk && callback.goalOk && callback.startOk && ros::ok())
    {      
      if(callback.cmdCntr < callback.cmdNum.data - 1)
      {
        //printf("command counter = %i\n", callback.cmdCntr);
        callback.startPose = callback.projection(callback.startPose, callback.cmdArr[callback.cmdCntr]);
        callback.cmdCntr++;
        nodeMarker = markerPoint(callback.startPose, callback.cmdCntr,1);
        callback.pubMarker.publish(nodeMarker);
      }
      else if(pointDistance(callback.startPose.position, callback.goal) > tolerance)
      {
        printf("too far from goal. replanning...\n");
        callback.replan();
      }
      else if(callback.cmdCntr == callback.cmdNum.data - 1)
      {
        //printf("restarting\n");
        callback.cmdCntr = callback.cmdStart;
        callback.startPose = callback.startPoseBackup;
      }
      if(callback.cmdStart == callback.cmdNum.data && pointDistance(callback.currentOdom.pose.pose.position, callback.goal) < 1.0)
      {
        printf("goal reached. \n");
        callback.deleteAllObstacles();
        
        callback.cmdOk = false;
        callback.cmdNumOk = false;
        callback.mapOk = false;
        callback.goalOk = false;
        callback.startOk = false;
        
        callback.cmdCallbackCntr = 0;
        callback.cmdCntr = 0;
        callback.cmdStart = -1;
        callback.cmdNum.data = -1;
        callback.pubMarker.publish(clearMarker());
      }
      if(!callback.obstacleProjection()){
        printf("collision with dynamic obstacle. replanning...\n");
        callback.replan();
      }
      ros::spinOnce();
    }
  }
  
  return 0;
}

//----Constructors----
//--------------------
callback::callback(ros::NodeHandle nh){
  printf("constructing callback class...\n");
  cmdOk = false;
  cmdNumOk = false;
  mapOk = false;
  goalOk = false;
  startOk = false;
  
  cmdCallbackCntr = 0;
  cmdCntr = 0;
  cmdStart = -1;
  cmdNum.data = -1;
  
  obstacleCounter = 0;
  tempObstacle = new dynamicObstacle;
  headObstacle = tempObstacle;
  headObstacle->id = 0;
  tempObstacle = new dynamicObstacle;
  tempObstacle->id = -1;
  headObstacle->next = tempObstacle;
  
  subOdom = nh.subscribe("/robot_0/odom", 1, &callback::odomCallback, this);
  //subOdom = nh.subscribe("/robot_0/base_pose_ground_truth", 1, &callback::odomCallback, this);
	subMap = nh.subscribe("/robot_0/robot_map/robot_map/costmap", 10, &callback::mapCallback, this);
	subGoal = nh.subscribe("/map_goal", 10, &callback::goalCallback, this);
  subCmdNum = nh.subscribe("/cmdNum", 10, &callback::cmdNumCallback, this);
	subCmd = nh.subscribe("/cmds", 500, &callback::cmdCallback, this);
	//subStartPose = nh.subscribe("/robot_0/cmd_vel", 10, &callback::startPoseCallback, this);
	subStartPose = nh.subscribe("/rrt_cmd", 10, &callback::startPoseCallback, this);
	subReplan = nh.subscribe("/rrt_replan", 1, &callback::replanCallback, this);
	subObstacle = nh.subscribe("/tracking", 10, &callback::obstacleCallback, this);
	
	pubReplan = nh.advertise<std_msgs::Bool>("/rrt_replan", 5, false);
  pubMarker = nh.advertise<visualization_msgs::Marker>("/treepoints",50,false);	
}

//-----Callbacks------
//--------------------
void callback::odomCallback(const nav_msgs::Odometry& msg){
  currentOdom = msg;
}

void callback::mapCallback(const nav_msgs::OccupancyGrid& msg){
  map = msg;
  mapOk = true;
  printf("Map received.\n");
}

void callback::goalCallback(const geometry_msgs::Point& msg){
  goal = msg;
  goalOk = true;
  printf("Goal received.\n");
}

void callback::cmdNumCallback(const std_msgs::Int16& msg){
  cmdNum = msg;
  cmdArr = new geometry_msgs::Twist[cmdNum.data];
  cmdNumOk = true;
  printf("receiving %i commands...\n", cmdNum.data);
}

void callback::cmdCallback(const geometry_msgs::Twist& msg){
  if(cmdNumOk){
    cmdArr[cmdCallbackCntr] = msg;
    printf("Received cmd #%i\n", cmdCallbackCntr);
    cmdCallbackCntr++;
  }
  if(cmdCallbackCntr == cmdNum.data){
    printf("All commands received.\n");
    cmdOk = true;
  }
}

void callback::startPoseCallback(const geometry_msgs::Twist& msg){
  if(cmdOk){
    startPose = currentOdom.pose.pose;
    startPoseBackup = startPose;
    cmdStart++;
    printf("cmdStart = %i\n", cmdStart);
    cmdCntr = cmdStart;
    startOk = true;
  }
}

void callback::replanCallback(const std_msgs::Bool& msg){
  if(msg.data){
    printf("received replan command\n");
    cmdOk = false;
    cmdNumOk = false;
    startOk = false;
  
    cmdCallbackCntr = 0;
    cmdCntr = 0;
    cmdStart = -1;
    cmdNum.data = -1;
  }
}

void callback::obstacleCallback(const nav_msgs::Odometry& msg){
  printf("RECEIVING OBSTACLE INFO\n");
  int id = (int)(msg.twist.twist.linear.z);      //dumb workaround to tell the id to the receiving Node. TODO find more elegant solution. use srv? -> call and response
  bool exist = false;
  tempObstacle = headObstacle->next;
  
  while(true){
    if(tempObstacle->id == -1)
      break;
    if(tempObstacle->id == id){
      exist = true;
      break;
    }
    tempObstacle = tempObstacle->next;
  }
  
  if(exist){
    printf("updating obstacle #%i\n", id);
    tempObstacle->cmd = msg.twist.twist;
    tempObstacle->startPose = msg.pose.pose;
  }
  
  else{
    printf("begin tracking obstacle #%i\n", id);
    obstacleCounter++;
    tempObstacle = new dynamicObstacle;
    tempObstacle->id = id;
    tempObstacle->cmd = msg.twist.twist;
    tempObstacle->startPose = msg.pose.pose;
  
    tempObstacle->next = headObstacle->next;
    headObstacle->next = tempObstacle;
  }
}

//-----Functions------
//--------------------

geometry_msgs::Pose callback::projection(geometry_msgs::Pose startPose, geometry_msgs::Twist cmd){
  //takes starting Pose and calcs projection
  geometry_msgs::Pose endPose = startPose;
  float v = cmd.linear.x;
  float w = cmd.angular.z;
  float alpha1 = tf::getYaw(startPose.orientation);
  float Ts = 0.1f;
  collisionCheck(endPose.position);
  for(int n = 1; n<=10; n++){
    if(w != 0){
      endPose.position.x = startPose.position.x - v/w * sin(alpha1) + v/w * sin(alpha1 + w*Ts*n);
      endPose.position.y = startPose.position.y + v/w * cos(alpha1) - v/w * cos(alpha1 + w*Ts*n);
    }
    else{
      endPose.position.x = startPose.position.x + n * v * Ts * cos(alpha1);
      endPose.position.y = startPose.position.y + n * v * Ts * sin(alpha1);
    }
    
    if(collisionCheck(endPose.position)){
      //publish to replan
      printf("path blocked. replanning...\n");
    }
  }
  
  float alpha2 = alpha1 + w*Ts*10;

  tf::Quaternion q_end = tf::createQuaternionFromRPY(0,0,alpha2);
  q_end.normalize();
	tf::quaternionTFToMsg(q_end, endPose.orientation);
  
  return endPose;
}

bool callback::collisionCheck(geometry_msgs::Point point){
  float x = point.x;
  float y = point.y;
  bool collision = false;
  
  //convert collision to costmap coordinates
  x -= map.info.origin.position.x;
  y -= map.info.origin.position.y;
  
  //convert coordinates to cells
  int cellx = x/map.info.resolution;
  int celly = y/map.info.resolution;

  //check value at point
  //printf("cellx = %i    celly = %i\n", cellx, celly);
  float value = map.data[cellx + celly*map.info.width];
  
  //printf("value: %f\n", value);
  if(value > 11.0){
    return true;
    //printf("collision detected\n");
  }
  return collision;
}

float pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

bool callback::obstacleProjection(){
  float minDistance = 1.0f;
  geometry_msgs::Pose tempPose;
  tempObstacle = headObstacle->next;
  while(true){
    if(tempObstacle->id != -1){
      printf("obstacle projection id = %i\n", tempObstacle->id);
      int steps = cmdCntr - cmdStart;
      float v = tempObstacle->cmd.linear.x;
      float w = tempObstacle->cmd.angular.z;
      float alpha1 = tf::getYaw(tempObstacle->startPose.orientation);
      
      if(abs(w) > 0.05){
        tempPose.position.x = tempObstacle->startPose.position.x - v/w * sin(alpha1) + v/w * sin(alpha1 + w*steps);
        tempPose.position.y = tempObstacle->startPose.position.y + v/w * cos(alpha1) - v/w * cos(alpha1 + w*steps);
      }
      else{
        tempPose.position.x = tempObstacle->startPose.position.x + steps * v * cos(alpha1);
        tempPose.position.y = tempObstacle->startPose.position.y + steps * v * sin(alpha1);
      }
      pubMarker.publish(markerPoint(tempPose, (steps)*2+tempObstacle->id, 2));
      
      if(pointDistance(tempPose.position, startPose.position) < minDistance){
        return false;  
      }
      tempObstacle = tempObstacle->next;
    }
    else
      break;
  }
  return true;
}

void callback::deleteAllObstacles(){
  tempObstacle = headObstacle->next;
  while(true){
    if(tempObstacle->id != 0 && tempObstacle->id != -1){
      printf("deleting obstacle #%i\n", tempObstacle->id);
      headObstacle->next = tempObstacle->next;
      free(tempObstacle);
      tempObstacle = headObstacle->next;
    }
    else
      break;
  }
}

//-----Publisher------
//--------------------

void callback::replan(){
  std_msgs::Bool replanNow;
  replanNow.data = true;
  pubReplan.publish(replanNow);
  cmdNumOk = false;
  startOk = false;
 
  cmdCallbackCntr = 0;
  cmdCntr = 0;
  cmdStart = -1;
  cmdNum.data = -1;
}

//---Visualization----
//--------------------

visualization_msgs::Marker markerPoint(geometry_msgs::Pose pose, int id, int type){
  
	visualization_msgs::Marker nodeMarker;
	nodeMarker.id = visualization_msgs::Marker::SPHERE;
	
	nodeMarker.header.frame_id = "map";
	nodeMarker.ns = "forward_projection";
	nodeMarker.id = id;

	nodeMarker.action = visualization_msgs::Marker::ADD;
	
	nodeMarker.color.g = 1.0f;
	nodeMarker.color.a = 1.0f;
	
	nodeMarker.scale.x = 0.2f;
	nodeMarker.scale.y = 0.2f;
	nodeMarker.scale.z = 0.01f;
	
	nodeMarker.pose = pose;
	nodeMarker.pose.position.z = 0.01f;
	nodeMarker.pose.orientation.w = 1.0f;
	
	if(id==0){
		nodeMarker.color.r = 1.0f;
		nodeMarker.color.b = 0.0f;
	}
	if(id==1){
		nodeMarker.color.g = 1.0f;
		nodeMarker.color.b = 0.0f;
	}
	if(type == 2){
	  nodeMarker.ns = "dynamic_obstacle";
	  nodeMarker.color.g = 0.0f;
	  nodeMarker.color.r = 1.0f;
	}
	return nodeMarker;	
}

visualization_msgs::Marker clearMarker(){

	visualization_msgs::Marker deleteAll;
	deleteAll.action = 3;
	return deleteAll;

}

