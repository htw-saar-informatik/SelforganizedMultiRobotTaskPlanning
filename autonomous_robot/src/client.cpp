/***
 *  Use this client so send x-y coordinates for the positons form command line 
 * 
 */


#include "ros/ros.h"
#include "autonomous_robot/AddNewTransportTask.h"
#include <cstdlib>
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <string>
#include <cstdint>

void callService(autonomous_robot::TransportTask task,ros::NodeHandle& n);

void printTask(autonomous_robot::TransportTask task){
    std::cout << "Task start [" << task.startPoint.x << "," << task.startPoint.y << "] ";
    std::cout << " end [" << task.endPoint.x << "," << task.endPoint.y << "]" << std::endl;
}

void info(std::string s){
    ROS_INFO_STREAM(s << std::endl);
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
    const int ARGUMENT_AMOUNT = 5;
    geometry_msgs::Pose2D startPose;
    geometry_msgs::Pose2D endPose;
    // autonomous_robot::TransportTask task;
    autonomous_robot::TransportTask task;
    
    //Check arguments
    if ( argc != ARGUMENT_AMOUNT ){
        std::cout << "Usage: client startpoint_x startpoint_y endpoint_x endpoint_y" << std::endl;
        return 1;
    }else{
        //Convert string parameters to long long values (float64bit)
        task = createTestTask( atoll(argv[1]) ,atoll(argv[2]), atoll(argv[3]) , atoll(argv[4]) );
    }

    ros::init(argc, argv, "client");
        

    ros::NodeHandle n;
    
    callService(task,n);

    return 0;
}

void callService(autonomous_robot::TransportTask task,ros::NodeHandle& n){
    ros::ServiceClient client = n.serviceClient<autonomous_robot::AddNewTransportTask>("AddNewTransportTask");
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
