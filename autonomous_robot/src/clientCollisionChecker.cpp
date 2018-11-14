/**
 * This client listens to the published positions of robots, and checks for each publishing if the current locations of the robots intersect
 * 
 */ 

#include "ros/ros.h"
#include "autonomous_robot/AddNewTransportTask.h"
#include <cstdlib>
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <string>
#include <cstdint>
#include <random>
#include <chrono>
#include <thread>
#include "autonomous_robot/RobotPosition.h"
#include <map>
void checkIfRobotsIntersect(autonomous_robot::RobotPosition position);

void sleep(int millis){
     std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

int intersectionCounter = 0;


/**
 * Checks if two circles intersect/touch
 */ 
int circle(int x1, int y1, int x2,  
           int y2, int r1, int r2) 
{ 
    int distSq = (x1 - x2) * (x1 - x2) + 
                 (y1 - y2) * (y1 - y2); 
    int radSumSq = (r1 + r2) * (r1 + r2); 
    if (distSq == radSumSq) 
        return 1; 
    else if (distSq > radSumSq) 
        return -1; 
    else
        return 0; 
} 


std::map<std::string,autonomous_robot::RobotPosition> map;

void positionChangeCallback(autonomous_robot::RobotPosition position){
    // std::cout << position.from <<  " " << position.radius << " " <<  position.location.x << " " << position.location.y << std::endl;

    map[position.from] = position;
    checkIfRobotsIntersect(position);

}
void checkIfRobotsIntersect(autonomous_robot::RobotPosition position){
    bool intersect = false;
    for (auto const& x : map){
        if( x.first.compare(position.from) != 0){
            // std::cout << position.from <<  " " << position.radius << " " <<  position.location.x << " " << position.location.y;
            // std::cout << "  ->  ";
            //  std::cout << x.second.from <<  " " << x.second.radius << " " <<  x.second.location.x << " " << x.second.location.y << std::endl;
            if( circle(x.second.location.x,x.second.location.y,position.location.x,position.location.y,x.second.radius,position.radius) > 0 ){
                intersect = true;
                intersectionCounter +=1;
            }
        }
    }
    if( intersect ){
        std::cout << "Intersect! " << intersectionCounter << std::endl;
    }
}


int main(int argc, char **argv) {
    geometry_msgs::Pose2D startPose;
    geometry_msgs::Pose2D endPose;
    // autonomous_robot::TransportTask task;
    
    std::cout << "Checking for intersections" << std::endl;


    ros::init(argc, argv, "clientCollisionChecker");
    ros::NodeHandle n;
   
    ros::Subscriber sub = n.subscribe("RobotPositionChange",1000,positionChangeCallback);

    ros::spin();
    return 0;
}




