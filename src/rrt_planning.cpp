#include "rrt_planning.h"
#include <tf/transform_datatypes.h>
#include <angles/angles.h>


int main(int argc, char **argv){
	ros::init(argc, argv, "rrt_planning") ;
  ros::NodeHandle nh;
  tree rrt(nh);
  ros::Rate spinRate(10);
  ros::Rate r(100000);
  
  srand(time(NULL));
  rrt.clearMarker();
  
	while(ros::ok()){
		ros::spinOnce();
		
		if(rrt.initialPoseFound && rrt.goalFound && rrt.mapFound){
		  printf("starting rrt...\n");
		  
			while(rrt.pointsInTree < rrt.maxPoints-1 && ros::ok()){
				rrt.generatePoint();
				//r.sleep();
				//usleep(5000);
				
				if(rrt.pathFound){
					printf("path found\n");
					break;
				}
			}
		}
		
    if(rrt.pathFound){
      printf("sending commands...\n");
      rrt.pathCmd();
      
      if(rrt.pathFound)
        break;
    }
    
	  if(rrt.pointsInTree >= rrt.maxPoints-1){
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

void tree::currentPoseCallback(const nav_msgs::Odometry& msg){
  if(initialPoseFound == false){
		treePoints[1].id = 1;
		treePoints[1].pose = msg.pose.pose;
		treePoints[1].cost = 0.0f;
		
		treePoints[1].cmd.linear.x = 0.0f;
		treePoints[1].cmd.linear.y = 0.0f;
		treePoints[1].cmd.linear.z = 0.0f;
		treePoints[1].cmd.angular.x = 0.0f;
		treePoints[1].cmd.angular.y = 0.0f;
		treePoints[1].cmd.angular.z = 0.0f;
    
    initialPoseFound = true;
    printf("Initial Position is set to x=%f and y=%f\n", treePoints[1].pose.position.x, treePoints[1].pose.position.y);
    markerPoint(treePoints[1].pose, 1);
  }
}

void tree::goalCallback(const geometry_msgs::Point& msg){
	if(goalFound == false){
		treePoints[0].id = 0;
		treePoints[0].pose.position = msg;
		treePoints[0].cost = 0.0f;
		
		treePoints[0].cmd.linear.x = 0.0f;
		treePoints[0].cmd.linear.y = 0.0f;
		treePoints[0].cmd.linear.z = 0.0f;
		treePoints[0].cmd.angular.x = 0.0f;
		treePoints[0].cmd.angular.y = 0.0f;
		treePoints[0].cmd.angular.z = 0.0f;
		
		goalFound = true;
		printf("Goal set to x=%f and y=%f\n", treePoints[0].pose.position.x, treePoints[0].pose.position.y);
    markerPoint(treePoints[0].pose, 0);
	}
}

void tree::replanCallback(const std_msgs::Bool& msg){
  if(msg.data == true){
    pointsInTree = 2;
	  initialPoseFound = false;
	  pathFound = false;
  }
}

//-------Functions--------
//------------------------

float tree::distance(geometry_msgs::Point point1, geometry_msgs::Point point2){
	float distance = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
	return distance;
}

void tree::generatePoint(){
  printf("generating Point...\n");
	geometry_msgs::Point goal = treePoints[0].pose.position;
	geometry_msgs:: Point origin = treePoints[1].pose.position;
	bool closeEnough = false;
	bool connectionSuccess = false;
	int cmdSuccess;
	
	geometry_msgs::Point sampledPoint;
	
	float tempDistance;
	int closestId;
	float closestDistance = 10000.0f;
	
	//data & calculations for sampling bias
	float distanceX = goal.x - origin.x;
	float distanceY = goal.y - origin.y;
	if(abs(distanceX) < 5)
	  distanceX = 5;
	if(abs(distanceY) < 5)
	  distanceY = 5;
	
	int invX = 1, invY = 1; 
	if(distanceX < 0)
	  invX = -1;
	if(distanceY < 0)
	  invY = -1;
	
	
	
	while(!connectionSuccess){
		closeEnough = false;
		
		//sampling Point and looking if it's close enough
		while(!closeEnough){
		  sampledPoint = origin;
			sampledPoint.x += (rand() % (int)(2*distanceX*1000)) / 1000.0 * invX - distanceX / 2;
			sampledPoint.y += (rand() % (int)(2*distanceY*1000)) / 1000.0 * invY - distanceY / 2;
			
		
			for(int n = 1; n<pointsInTree; n++){
				tempDistance = distance(treePoints[n].pose.position, sampledPoint);
			
				if(tempDistance < closestDistance){
					closestDistance = distance(treePoints[n].pose.position, sampledPoint);
					closestId = n;
			
					if(closestDistance<maxPointDistance){
						closeEnough = true;
					}
				}
			}
		}

		cmdSuccess = generateCommand(sampledPoint, closestId);
		
		if(cmdSuccess==0){
		  printf("closest Point: #%i\n", closestId);
			connectionSuccess = true;
			treePoints[pointsInTree].id = pointsInTree;
			treePoints[pointsInTree].parentId = treePoints[closestId].id;
			
			//markerPoint(treePoints[pointsInTree].pose, pointsInTree);
      //markerList(pointsInTree);
		  
		  if(distance(treePoints[pointsInTree].pose.position, treePoints[0].pose.position)<0.5){
		    pathFound = true;
		    treePoints[0].parentId = pointsInTree;
		    printf("%i\n", treePoints[0].parentId);
		    //createPath();
		  }
			pointsInTree++;
		}
	}
}

int tree::generateCommand(geometry_msgs::Point goal, int startId){
	geometry_msgs::Twist tempCmd;
	geometry_msgs::Pose tempPose;
	float tempCost;
	
	float closestDistance = 1000.0f;
	float tempDistance = 0.0f;
	int success = -1;
	
	int samplingNumber = 10;
	float speed = 1.0f;
	float angMax = 1.0f;
	
	tempCmd.linear.x = speed;
	tempCmd.linear.y = 0.0f;
	tempCmd.linear.z = 0.0f;
	tempCmd.angular.x = 0.0f;
	tempCmd.angular.y = 0.0f;
	
	
	
	for(int n = 0; n<samplingNumber; n++){
		tempCmd.angular.z = (double)(rand() % 1001 -500)/1001*angMax;
		
		if(cmdIntegration(tempCmd.linear.x, treePoints[startId].pose, tempCmd, &tempPose, &tempCost)==0){
			tempDistance = distance(goal, tempPose.position);
		
			if(tempDistance < closestDistance){
				closestDistance = tempDistance;
				
				treePoints[pointsInTree].pose = tempPose;
				treePoints[pointsInTree].cmd = tempCmd;
				treePoints[pointsInTree].cost = treePoints[startId].cost + tempCost;
				
				success = 0;
			}
		}
	}	
	return success;
}

int tree::cmdIntegration(float speed, geometry_msgs::Pose start, geometry_msgs::Twist cmd, geometry_msgs::Pose* endPtr, float* costPtr){
  //Inputs: startPose, cmd, ptr to endPose and cost
  //calcs endPoint from startPose and cmd and saves it in endPose
  //also check Points along path for collision
  float w = cmd.angular.z;
  float v = cmd.linear.x;
  float alpha1 = tf::getYaw(start.orientation);
  float Ts = 0.2;
  geometry_msgs::Point point;
  
  for(int n = 1; n<=1/Ts; n++){
    if( w != 0){
      point.x = start.position.x - v/w * sin(alpha1) + v/w * sin(alpha1 + w*Ts*n);
      point.y = start.position.y + v/w * cos(alpha1) - v/w * cos(alpha1 + w*Ts*n);
    }
    else{
      point.x = start.position.x + n * v * Ts * cos(alpha1);
      point.y = start.position.y + n * v * Ts * sin(alpha1);
    }
    
    if(collisionCheck(point))
      return -1;
  }
  
  float alpha2 = tf::getYaw(start.orientation) + w;
  
  geometry_msgs::Pose endPose;
  endPtr->position.x = point.x;
  endPtr->position.y = point.y;
  *costPtr = 1;
  
  tf::Quaternion q_end = tf::createQuaternionFromRPY(0,0,alpha2);
  q_end.normalize();
	tf::quaternionTFToMsg(q_end, endPtr->orientation);
	
  return 0;
}

bool tree::collisionCheck(geometry_msgs::Point point){
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
  float value = map.data[cellx + celly*map.info.width];
  
  if(value > 20.0){
    return true;
    //printf("collision detected\n");
  }
  return collision;
}

geometry_msgs::Point tree::cellToCoord(int cell){
  geometry_msgs::Point coord;
  int cellX = cell % map.info.width + 1;
  int cellY = cell / map.info.height - 1;
  
  coord.x = cellX * map.info.resolution + map.info.origin.position.x;
  coord.y = cellY * map.info.resolution + map.info.origin.position.y;
  
  return coord;
}

int tree::coordToCell(geometry_msgs::Point coord){
  float cell = 0;

  cell += (coord.x - map.info.origin.position.x) / map.info.resolution;
  cell += (coord.y - map.info.origin.position.y) / map.info.resolution * map.info.width;
  
  return (int)(cell);
}

void tree::pathCmd(){
	ros::Rate pubRate(1);
  treePose pathCmd = treePoints[treePoints[0].parentId];
	std_msgs::Int16 cmdNum;
	
  cmdNum.data = 0;
  
  ros::Time start = ros::Time::now();
  
  //count commands
	while(pathCmd.id != 0){
	  cmdNum.data++;
	  pathCmd = treePoints[pathCmd.parentId];
	}
	
	pubCmdNum.publish(cmdNum);
	pathCmd = treePoints[treePoints[0].parentId];
	printf("%i cmds in path\n", cmdNum.data);	
  int tempId = 1;
  usleep(100);
  
  //send all cmds to forward Projection as fast as possible
  while(pathCmd.id != 0){
			
		while(pathCmd.parentId != tempId){
			pathCmd = treePoints[pathCmd.parentId];
		}
		
		tempId = pathCmd.id;
		pubProjection.publish(treePoints[tempId].cmd);
		usleep(100);
	}
	
  float duration = ros::Time::now().toSec() - start.toSec();
  printf("cmds sent to forward projection. duration = %f\n", duration); 
  pathCmd = treePoints[treePoints[0].parentId]; 
  tempId = 1;
  
  //send commands in specified rate to robot
  ros::Time cycleStart;
	while(pathCmd.id != 0){
			
		while(pathCmd.parentId != tempId){
			pathCmd = treePoints[pathCmd.parentId];
		}
				
		tempId = pathCmd.id;
			
		pubRate.sleep();
		pubCmd.publish(treePoints[tempId].cmd);
		
		cycleStart = ros::Time::now();
		while(ros::Time::now().toSec() - cycleStart.toSec() < 0.9){
		  ros::spinOnce();
		  if(!pathFound){
		  	pubCmd.publish(treePoints[0].cmd);
		    break;
		  }
		}
		if(!pathFound)
		  break;
	}
	
	if(pathFound){
	  usleep(100);
	  pubCmd.publish(treePoints[0].cmd);
	  printf("sending complete.\n");
	}
	else
	  printf("replanning...\n");
}
























