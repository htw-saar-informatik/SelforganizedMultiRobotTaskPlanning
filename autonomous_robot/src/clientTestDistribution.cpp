/**
 * Client to test the task distribution (not the execution of the tasks)
 */ 

#include "ros/ros.h"
#include "autonomous_robot/AddNewTransportTask.h"
#include <cstdlib>
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <string>
#include <cstdint>
#include <random>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>

std::string FILE_NAME = "./testfile";

void callService(autonomous_robot::TransportTask task,ros::NodeHandle& n);

void printTask(autonomous_robot::TransportTask task){
    std::cout << "Task start [" << task.startPoint.x << "," << task.startPoint.y << "] ";
    std::cout << " end [" << task.endPoint.x << "," << task.endPoint.y << "]" << std::endl;
}

void info(std::string s){
    ROS_INFO_STREAM(s << std::endl);
}

std::string getFilePath (const std::string& str){
  std::size_t found = str.find_last_of("/\\");
  return str.substr(0,found);
  //str.substr(found+1) would be only file name without path
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

int getRandomBetween(int min, int max){
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_int_distribution<int> dist(min, max);
    return dist(engine);
}

std::vector<std::string> split(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

void generateTasks(int amount){
    std::stringstream ss;
    for( int i = 0; i < amount ; i++){
        ss << getRandomBetween(0,100) << " " << getRandomBetween(0,100) << " " << getRandomBetween(0,100) << " " << getRandomBetween(0,100) << std::endl;
    }
    std::cout << ss.str();
}

int randomFrom;
int randomTo;
std::vector<autonomous_robot::TransportTask> tasks;
void readFile(std::string testname){
    bool startFound = false;
    std::ifstream infile(FILE_NAME);
    std::string line;
        while ( std::getline(infile,line)){
            if ( line.empty() ){
                if (startFound){
                    startFound = false;
                }
            }else{
                std::vector<std::string> splitted = split(line,' ');
                
                if ( splitted[0].compare("-") == 0){

                    if ( splitted[1].compare(testname) == 0 ){
                        startFound = true;
                    } 
                }else{
                    if ( startFound ){ 
                        if( splitted[0].compare("w") == 0 ){
                            //set waiting times for random
                            randomFrom = stoi(splitted[1]);
                            randomTo = stoi(splitted[2]);
                            std::cout << "W found " << randomFrom << ", " << randomTo << std::endl;
                        }else if ( splitted[0].compare("r") == 0){
                            //Do nothing here
                        }else{
                            //Its an task, parse it
                            tasks.push_back(createTestTask(stoi(splitted[0]),stoi(splitted[1]),stoi(splitted[2]),stoi(splitted[3])));
                        }
                    }
                }
            }

            
        }

}

int totalTime = 0;

int main(int argc, char **argv) {
    geometry_msgs::Pose2D startPose;
    geometry_msgs::Pose2D endPose;
    // autonomous_robot::TransportTask task;
    
    FILE_NAME = std::string(getFilePath(argv[0]) + "/testfile");
    std::cout << FILE_NAME << std::endl;
    ros::init(argc, argv, "clientTestDistribution");
        

    ros::NodeHandle n;
    
    if (argc < 2 ){
        std::cout << "Usage: clientTestDistribution testcasename" << std::endl;
        exit(1);
    }

    readFile(std::string(argv[1]));
    int totalDistanceForTasks = 0;
    for ( int i = 0; i < tasks.size(); i++){
        printTask(tasks[i]);
        totalDistanceForTasks +=  abs(tasks[i].startPoint.x - tasks[i].endPoint.x) + abs(tasks[i].startPoint.y-tasks[i].endPoint.y);
    }
    std::cout << "Distanz für die Aufträge/Ausführungszeit: " << totalDistanceForTasks << std::endl;

    //Call all tasks
    while( tasks.size() != 0){
        int rnd = getRandomBetween(0,tasks.size()-1);
        callService(tasks[rnd],n);
        tasks.erase(tasks.begin()+rnd);
    }
    // for( int i = 0; i < tasks.size();i++){
    //     callService(tasks[i],n);
    // }

    std::cout << "TotalTime: " << totalTime << std::endl;
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

    //Wait random time till next request
    int time = getRandomBetween(randomFrom,randomTo);
    totalTime+= time;
    std::cout << "Wait for " << time << "s" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(time));
}
