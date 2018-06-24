#include "rrt_planning_libnabo.h"
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rrt_planning") ;
  ros::NodeHandle nh;
  tree rrt(nh);
  ros::Rate spinRate(10);
  ros::Rate r(100000);
  
  srand(time(NULL));
  rrt.clearMarker();
  
	while(ros::ok())
	{
		ros::spinOnce();
		if(rrt.initialPoseFound && rrt.goalFound && rrt.mapFound)
		{
		  printf("starting rrt...\n");
		  ros::Time rrtBegin = ros::Time::now();
			while(rrt.pointsInTree < rrt.maxPoints-1 && ros::ok())
			{
				rrt.generatePoint();
			  //usleep(5000); //markerDelay
				if(rrt.pathFound)
				{
				  double secs = (ros::Time::now().toNSec() - rrtBegin.toNSec())*1e-9;
					printf("path found. time = %fs\n", secs);
					break;
				}
			}
		}
		
    if(rrt.pathFound)
    {
      printf("sending commands...\n");
      rrt.pathCmd();
      if(rrt.pathFound)
        break;
    }
	  if(rrt.pointsInTree >= rrt.maxPoints-1)
	  {
	    printf("maxPoints exceeded\n");
	    break;
	  }
		spinRate.sleep();
	}
  return 0;
}

//---Callback functions---
//------------------------

void tree::mapCallback(const nav_msgs::OccupancyGrid& msg){
	map = msg;
	
  mapFound = true;
  printf("map detected\n");
}

void tree::initialPoseCallback(const nav_msgs::Odometry& msg){
  if(initialPoseFound == false){
    treePositions(0,0) = msg.pose.pose.position.x;
    treePositions(1,0) = msg.pose.pose.position.y;
    treeCmds[0].orientation = msg.pose.pose.orientation;
		treeCmds[0].id = 0;
		treeCmds[0].cost = 0.0f;
		
		treeCmds[0].cmd.linear.x = 0.0f;
		treeCmds[0].cmd.linear.y = 0.0f;
		treeCmds[0].cmd.linear.z = 0.0f;
		treeCmds[0].cmd.angular.x = 0.0f;
		treeCmds[0].cmd.angular.y = 0.0f;
		treeCmds[0].cmd.angular.z = 0.0f;
    
    initialPoseFound = true;
    printf("Initial Position is set to x=%f and y=%f\n", treePositions(0,0), treePositions(1,0));
    markerPoint(treePositions.col(0), 0);
  }
}

void tree::goalCallback(const geometry_msgs::Point& msg){
	if(goalFound == false){
		goal(0) = msg.x;
		goal(1) = msg.y;
	}
}

void tree::replanCallback(const std_msgs::Bool& msg){
  if(msg.data == true){
    pointsInTree = 1;
	  initialPoseFound = false;
	  pathFound = false;
  }
}

//-------Functions--------
//------------------------
float tree::vectorDistance(Eigen::Vector2f v, Eigen::Vector2f w){
  return sqrt(pow(v(0)-w(0),2)+pow(v(1)-w(1),2));
}

void tree::generatePoint(){
  printf("generating Point...\n");
	bool closeEnough = false;
	int cmdSuccess = -1;
  int connectionAttempts = 0;
  const int K = 1;
  Eigen::VectorXi indices(K);
  Eigen::VectorXf dists2(K);
  Eigen::VectorXf sampledPoint(2);
  Nabo::NNSearchF* nns = Nabo::NNSearchF::createKDTreeTreeHeap(treePositions);
  
  while(cmdSuccess != 0)
  {
    while(closeEnough = false)
    {      
      //sampling area is limited to roughly the size of corridor map
      sampledPoint(0) = (rand()%(int)(10.0*1000.0)) / 1000.0 - 10.0 / 2.0;
      sampledPoint(1) = (rand()%(int)(60.0*1000.0)) / 1000.0 - 60.0 / 2.0;
      nns->knn(sampledPoint, indices, dists2, K);
      
      if(sqrt(dists2(0)) < maxPointDistance)
      {
        closeEnough = true;
      }
    }
    cmdSuccess = generateCommand(sampledPoint, indices(0));
    connectionAttempts++;
  }
  printf("connection attempts needed: %i\n", connectionAttempts);
  
  
  if(cmdSuccess == 0){
    printf("closest Point: #%i\n", indices(0));
    treeCmds[pointsInTree].id = pointsInTree;
    treeCmds[pointsInTree].parentId = indices(0);
    
    treePositions(0, pointsInTree) = sampledPoint(0);
    treePositions(1, pointsInTree) = sampledPoint(1);
		
		//markerPoint(treeCmds[pointsInTree].pose, pointsInTree);
    //markerList(pointsInTree);
		  
		if(vectorDistance(goal, sampledPoint)<0.5){
		  pathFound = true;
		}
		else{
	    pointsInTree++;
    }
  }
}

int tree::generateCommand(Eigen::VectorXf sampledPoint, int startId){
	geometry_msgs::Twist tempCmd;
	Eigen::Vector2f tempPosition = treePositions.col(startId);
	geometry_msgs::Quaternion tempOrientation = treeCmds[startId].orientation;
	float tempCost;
	
	float closestDistance = 1000.0f;
	float tempDistance = 0.0f;
	int success = -1;
	
	int samplingNumber = 5;
	float speed = 1.0f;
	float angMax = 1.0f;
	
	tempCmd.linear.x = speed;
	tempCmd.linear.y = 0.0f;
	tempCmd.linear.z = 0.0f;
	tempCmd.angular.x = 0.0f;
	tempCmd.angular.y = 0.0f;	
	
	for(int n = 0; n<samplingNumber; n++){
		tempCmd.angular.z = (double)(rand() % 1001 -500)/1001*angMax;
		
		if(cmdIntegration(tempCmd, &tempOrientation, &tempPosition, &tempCost)==0){
			tempDistance = vectorDistance(sampledPoint, tempPosition);
		
			if(tempDistance < closestDistance){
				closestDistance = tempDistance;
				treePositions.col(pointsInTree) = tempPosition;
				treeCmds[pointsInTree].orientation = tempOrientation;
				treeCmds[pointsInTree].cmd = tempCmd;
				treeCmds[pointsInTree].cost = treeCmds[startId].cost + tempCost;
				
				success = 0;
			}
		}
	}	
	return success;
}

int tree::cmdIntegration(geometry_msgs::Twist cmd, geometry_msgs::Quaternion* orientation, Eigen::Vector2f* position, float* cost){
  //Inputs: startPose, cmd, ptr to endPose and cost
  //calcs endPoint from startPose and cmd and saves it in endPose
  //also check Points along path for collision
  float w = cmd.angular.z;
  float v = cmd.linear.x;
  float alpha1 = tf::getYaw(*orientation);
  float Ts = 0.2;
  Eigen::Vector2f checkPoint(2);
  
  for(int n = 1; n<=1/Ts; n++){
    if( w != 0){
      checkPoint(0) = (*position)(0) - v/w * sin(alpha1) + v/w * sin(alpha1 + w*Ts*n);
      checkPoint(1) = (*position)(1) + v/w * cos(alpha1) - v/w * cos(alpha1 + w*Ts*n);
    }
    else{
      checkPoint(0) = (*position)(0) + n * v * Ts * cos(alpha1);
      checkPoint(1) = (*position)(1) + n * v * Ts * sin(alpha1);
    }
    
    if(collisionCheck(checkPoint))
      return -1;
  }
  float alpha2 = alpha1 + w;
  tf::Quaternion q_end = tf::createQuaternionFromRPY(0,0,alpha2);
  q_end.normalize();
  
	tf::quaternionTFToMsg(q_end, *orientation);
	*position = checkPoint;
  *cost = 1;
  
  return 0;
}

bool tree::collisionCheck(Eigen::Vector2f point){
  float x = point(0);
  float y = point(1);
  bool collision = false;
  
  //convert collision to costmap coordinates
  x -= map.info.origin.position.x;
  y -= map.info.origin.position.y;
 
  //convert coordinates to cells
  int cellx = x/map.info.resolution;
  int celly = y/map.info.resolution;
  
  //check value at point
  float value = map.data[cellx + celly*map.info.width];
  
  if(value > 10.0){
    return true;
  }
  return collision;
}

Eigen::Vector2f tree::cellToCoord(int cell){
  Eigen::Vector2f coord;
  int cellX = cell % map.info.width + 1;
  int cellY = cell / map.info.height - 1;
  
  coord(0) = cellX * map.info.resolution + map.info.origin.position.x;
  coord(1) = cellY * map.info.resolution + map.info.origin.position.y;
  
  return coord;
}

int tree::coordToCell(Eigen::Vector2f coord){
  float cell = 0;

  cell += (coord(0) - map.info.origin.position.x) / map.info.resolution;
  cell += (coord(1) - map.info.origin.position.y) / map.info.resolution * map.info.width;
  
  return (int)(cell);
}

void tree::pathCmd(){
  treeCmd pathCmd = treeCmds[pointsInTree];
	std_msgs::Int16 cmdNum;
	
  cmdNum.data = 0;
  
  ros::Time start = ros::Time::now();
  
  //count commands
	while(pathCmd.id != 0){
	  cmdNum.data++;
	  pathCmd = treeCmds[pathCmd.parentId];
	}
	
	pubCmdNum.publish(cmdNum);
	pathCmd = treeCmds[pointsInTree];
	printf("%i cmds in path\n", cmdNum.data);	
  int tempId = 0;
  usleep(100);    //TODO try make it work without usleep
  
  //send all cmds to forward Projection as fast as possible. TODO maybe send first command to robot before sending to forward projection
  while(pathCmd.id != pointsInTree)
  {
		while(pathCmd.parentId != tempId)
			pathCmd = treeCmds[pathCmd.parentId];
		
		pubProjection.publish(treeCmds[pathCmd.id].cmd);
		tempId = pathCmd.id;
		usleep(100);//TODO another usleep to get rid of
	}
	
  float duration = ros::Time::now().toSec() - start.toSec();
  printf("cmds sent to forward projection. duration = %f\n", duration); 
  pathCmd = treeCmds[pointsInTree]; 
  tempId = 0;
  
  //send commands in specified rate to robot. first command sent before loop at time 0 of ros::Rate.
  while(pathCmd.parentId != tempId)
    pathCmd = treeCmds[pathCmd.parentId];
  ros::Time cycleStart;
  ros::Rate pubRate(1);
  pubCmd.publish(pathCmd.cmd);
  tempId = pathCmd.id;
  
	while(pathCmd.id != pointsInTree && ros::ok())
	{
		while(pathCmd.parentId != tempId)
			pathCmd = treeCmds[pathCmd.parentId];
		
		pubRate.sleep();
		pubCmd.publish(pathCmd.cmd);
		cycleStart = ros::Time::now();
		tempId = pathCmd.id;
		
		//checking for replan commands
		while(ros::Time::now().toSec() - cycleStart.toSec() < 0.9 && ros::ok())
		{
		  ros::spinOnce();
		  if(!pathFound)
		    break;
		}
		if(!pathFound)
		  break;
	}
	pubCmd.publish(treeCmds[0].cmd);
	if(pathFound)
	  printf("sending complete.\n");
	else
	  printf("replanning...\n");
}
























