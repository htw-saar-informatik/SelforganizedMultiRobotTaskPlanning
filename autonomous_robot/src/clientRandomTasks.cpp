/**
 * This client generates random transporttasks and sends them to the robots
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
void callService(autonomous_robot::TransportTask task);
autonomous_robot::TransportTask createTestTask(long long x1, long long y1, long long x2, long long y2);

geometry_msgs::Point startPoint;
geometry_msgs::Point endPoint;
bool startPointSet = false;
ros::NodeHandle* nodeHandle;

void printTask(autonomous_robot::TransportTask task){
    std::cout << "Task start [" << task.startPoint.x << "," << task.startPoint.y << "] ";
    std::cout << " end [" << task.endPoint.x << "," << task.endPoint.y << "]" << std::endl;
}


void sleep(int millis){
     std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

int getRandomBetween(int min, int max){
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_int_distribution<int> dist(min, max);
    return dist(engine);
}

autonomous_robot::TransportTask createTestTask(long long x1, long long y1, long long x2, long long y2){
    autonomous_robot::TransportTask task;
    geometry_msgs::Pose2D startPoint;
    geometry_msgs::Pose2D endPoint;
    startPoint.x = x1;
    startPoint.y = y1;
    
    endPoint.x = x2;
    endPoint.y = y2;
    task.endPoint = endPoint;
    task.startPoint = startPoint;
    return task;
}

int main(int argc, char **argv) {
    geometry_msgs::Pose2D startPose;
    geometry_msgs::Pose2D endPose;
   
    ros::init(argc, argv, "clientRandomTasks");
    ros::NodeHandle n;
    nodeHandle = &n;
   
    while(true){
        sleep(1000*getRandomBetween(2,7));

        double x1 = getRandomBetween(-50,50);
        double y1 = getRandomBetween(-50,50);
        double x2 = getRandomBetween(-50,50);
        double y2 = getRandomBetween(-50,50);

        autonomous_robot::TransportTask task = createTestTask(x1,y1,x2,y1);

        callService(task);

        
    }

    

    // callService(task,n);

    return 0;
}



void callService(autonomous_robot::TransportTask task){
    ros::ServiceClient client = nodeHandle->serviceClient<autonomous_robot::AddNewTransportTask>("AddNewTransportTask");
    autonomous_robot::AddNewTransportTask addNewTransportTaskService;
    addNewTransportTaskService.request.task = task;
    
    std::cout << "Send task:" << std::endl;
    printTask(task);

    if (client.call(addNewTransportTaskService))
    {
        std::int8_t success = addNewTransportTaskService.response.success;
        //std_msgs::Int8 success = addNewTransportTaskService.response.success;
        
        if( success == 0){
            ROS_INFO("New Task successfully added");    
        }else{
            ROS_INFO("There was an error : %d",success);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service AddNewTransportTask");
        exit(1);
    }
}
