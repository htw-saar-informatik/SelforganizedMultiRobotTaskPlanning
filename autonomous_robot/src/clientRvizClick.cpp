/**
 * This client listens to goals in rviz and sends them to the robots
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

void rvizCallback(geometry_msgs::PoseStamped::ConstPtr msg){
    double x =  std::round(msg->pose.position.x);
    double y =  std::round(msg->pose.position.y);
    
    if ( !startPointSet){
        startPoint.x = x;
        startPoint.y = y;
        std::cout << "Startpoint set to: [" << x << "," << y << "]" << std::endl;
        startPointSet = true;
    }else{
        endPoint.x = x;
        endPoint.y = y;
        std::cout << "Endpoint set to: [" << x << "," << y << "]" << std::endl;
    
        startPointSet = false;

        //Send task to masternode
        autonomous_robot::TransportTask task = createTestTask(startPoint.x,startPoint.y,endPoint.x,endPoint.y);
        callService(task);
    }

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
    // autonomous_robot::TransportTask task;
    autonomous_robot::TransportTask task;
    


    ros::init(argc, argv, "clientRvizClick");
    ros::NodeHandle n;
    nodeHandle = &n;
    ros::Subscriber sub = n.subscribe("move_base_simple/goal",1000,rvizCallback);

    ros::spin();

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
