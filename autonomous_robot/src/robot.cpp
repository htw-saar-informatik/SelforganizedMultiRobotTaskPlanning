#include "ros/ros.h"
#include "autonomous_robot/AddNewTransportTask.h"
#include "autonomous_robot/ReceiveOffer.h"
#include "autonomous_robot/ReceiveCounterOffer.h"
#include "autonomous_robot/ReceiveReachPoints.h"
#include "autonomous_robot/InformReachPoint.h"

#include "visualization_msgs/Marker.h"  //rviz marker
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include "TaskDistributer.h"
#include "OfferManager.h"
#include "MutualExclusion.h"
#include "SharedFunctions.h"
#include "RAI.h"
#include "CollisionPlanner/CollisionPlanner.h"
#include "geometry_msgs/Point.h"
//Exit code
const int WRONG_ARGUMENTS_COUNT = 1;
const int EXIT_NO_ROBOT_FILE_FOUND = 2;


//Global variables
std::string FILE_ROBOTS_LIST;
std::string FILE_CONFIG;
const std::string ROBOTS_FILE_NAME = "robots_list";
const std::string CONFIG_FILE_NAME = "config";
TaskDistributer distributer;
OfferManager offerManager;
MutualExclusion mutualExclusion;

//Method stubs
ros::ServiceServer createServerReceiveCounterOffer();
ros::ServiceServer createServerReceiveOffer();
ros::ServiceServer createServerAddNewTransportTask();
ros::ServiceServer createServerWinOffer();
ros::ServiceServer createServiceLooseOffer();
ros::ServiceServer createServiceRequestMutualExclusion();
ros::ServiceServer createServiceReleaseMutualExclusion();
ros::ServiceServer createServiceConfirmMutualExclusion();
ros::ServiceServer createServiceRequestCurrentExecutionPath();
ros::ServiceServer createServiceReceiveReachPoints();
ros::ServiceServer createServiceInformReachPoint();

autonomous_robot::TransportTask createTestTask(double x1, double y1, double x2, double y2,int id);
void publishPosition();
void testRAI();
void testCollisionPlanner();
void readConfig();

//std::string getServiceName(std::string nodeID, std::string serviceName);
void readRobotsFromFile();
bool existsFile (const std::string& name);
std::string getFilePath (const std::string& str);


// static autonomous_robot::TransportTask getCurrentExecutionTransportTask(){
//     return currentExecutionTransportTask;
// }

void testMutual(){
    sleep(5);
    //request mutual exclusion

    std::cout << "request mutual exclusion " << std::endl;

     ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<autonomous_robot::RequestMutualExclusion>("RequestMutualExclusion");
    autonomous_robot::RequestMutualExclusion requestMutualExclusionService;
    requestMutualExclusionService.request.nodeID = SharedFunctions::nodeID;

    if (client.call(requestMutualExclusionService)){
        ROS_INFO("Call for requestMutualExclison successful");
    }
    else
    {
        ROS_ERROR("Failed to call service RequestMutualExclusion to master");
    }

    //Wait here until the right for mutual exclusion is obtained
    std::cout << "Wait here until the right for mutual exclusion is obtained "  << SharedFunctions::nodeID << std::endl;
    std::unique_lock<std::mutex> lock(SharedFunctions::mutex_mutual_exclusion);
    SharedFunctions::condition_mutual_exclusion.wait(lock);
    std::cout << "I have obtained the right to do a mutual exclusion "  << SharedFunctions::nodeID  << std::endl;

    sleep(10);
    ROS_INFO("release critical section");
    mutualExclusion.sendReleaseMessage();

}

//Sets a marker in rviz
void setMarker(geometry_msgs::Point p){
    static ros::Publisher vis_pub = SharedFunctions::nodeHandler->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = SharedFunctions::frame_id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

     //size for CYLINDER
    marker.scale.x = SharedFunctions::radius*2;
    marker.scale.y = SharedFunctions::radius*2;
    marker.scale.z = 2;

    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = marker.scale.z/2;
    

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

   

    marker.color.a = 1.0; // Don't forget to set the alpha!
    // marker.color.r = 0.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;
    std::cout << SharedFunctions::color.r << " " << SharedFunctions::color.g << " " << SharedFunctions::color.b << std::endl;
    marker.color.r = SharedFunctions::color.r;
    marker.color.g = SharedFunctions::color.g;
    marker.color.b = SharedFunctions::color.b;

    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();


    // std::cout << "Publishing " << std::endl;
    vis_pub.publish( marker );
}


int main(int argc, char **argv){
    srand (static_cast <unsigned> (time(0)));
    bool isMasterNode = false;
    
    //Check parameters
    if ( argc < 5 ){
        std::cout << "Usage: robot name(String) frame_id(int) posX(int) posY(int) [m]" << std::endl;
        return WRONG_ARGUMENTS_COUNT;
    }else{
        //Set file name to read name of other robots, file has to be in same directory as this executable)
        FILE_ROBOTS_LIST = std::string(getFilePath(argv[0]) + "/" + ROBOTS_FILE_NAME);
        FILE_CONFIG = std::string(getFilePath(argv[0]) + "/" + CONFIG_FILE_NAME);
        //Set Name of this Node
        SharedFunctions::nodeID = argv[1];
        SharedFunctions::frame_id = atoi(argv[2]);
    }

    //Check if this node is a masternode
    if ( argc == 6 ){
        if ( argv[5] == std::string("m") ){
            ROS_INFO("Set this node as masternode");
            isMasterNode = true;
        }
    }

    SharedFunctions::currentPosition.x = atoll(argv[3]);
    SharedFunctions::currentPosition.y = atoll(argv[4]);

    //Set/Read current position of the robot in space
    //TODO: Get location for the robot in a real map
     std::cout << "Robot: " << SharedFunctions::nodeID << " frame_id: " << SharedFunctions::frame_id << " Startposition: [" << SharedFunctions::currentPosition.x << "," << SharedFunctions::currentPosition.y << "]" << std::endl;

    

    //Mark that there is currently no execution task
    SharedFunctions::currentExecutionTransportTask.id = -1;
    

     //Read from file known robots in the system
     std::vector<std::string> robotNames; //Does live until process terminates
     SharedFunctions::robotNames = &robotNames;
    readRobotsFromFile();
    //List Robots in file
    std::stringstream ss;
    for( std::string name: *SharedFunctions::robotNames){
          ss << name << " ";
    }
    
    ROS_INFO_STREAM("Found Robots: " << ss.str());
    
    readConfig();
   
    if( SharedFunctions::simulateDrivingForTests){
        ROS_INFO("Simultion: do not drive ");
    }else{
        ROS_INFO("Simulation: do drive");
    }

    //Create new vector to store all transport tasks to execute
    std::vector<autonomous_robot::TransportTask> acceptedTasks; //Does live until process terminates
    SharedFunctions::acceptedTasks = &acceptedTasks;

    //Create new vectore to store tasks order for new task
    std::vector<autonomous_robot::TransportTask> calculatedOrderForNewTask; //Does live until process terminates
    SharedFunctions::calculatedOrderForNewTask = &calculatedOrderForNewTask;

    ros::init(argc,argv,SharedFunctions::nodeID);
    ros::NodeHandle n;
    SharedFunctions::nodeHandler = &n;
    
    ros::Publisher position_publisher = n.advertise<autonomous_robot::RobotPosition>("RobotPositionChange",1000);
    SharedFunctions::position_publisher = &position_publisher;
    publishPosition();
    sleep(1);
    publishPosition();
    
    ros::ServiceServer serviceAddNewTransportTask;
    ros::ServiceServer serviceReceiveOffer;
    ros::ServiceServer serviceReceiveCounterOffer;
    ros::ServiceServer serviceWinOffer;
    ros::ServiceServer serviceLooseOffer;
    ros::ServiceServer serviceRequestMutualExclusion;
    ros::ServiceServer serviceConfirmMutualExclusion;
    ros::ServiceServer serviceReleaseMutualExclusion;
    ros::ServiceServer serviceRequestCurrentExecutionPath;
    ros::ServiceServer serviceReceiveReachPoints;
    ros::ServiceServer serviceInformReachPoint;

    if ( isMasterNode ){
       serviceAddNewTransportTask = createServerAddNewTransportTask();
       serviceReceiveCounterOffer = createServerReceiveCounterOffer();
       serviceRequestMutualExclusion = createServiceRequestMutualExclusion();
       serviceReleaseMutualExclusion = createServiceReleaseMutualExclusion();
        distributer.startTaskProcessing();
        mutualExclusion.processRequests();

         if( SharedFunctions::distributionMode == 0){
            ROS_INFO("Use distribution: distance");
        }else{
            ROS_INFO("Use distribution: execution time");
        }
    }

    //Set Service
    serviceReceiveOffer = createServerReceiveOffer();
    serviceWinOffer = createServerWinOffer();
    serviceLooseOffer = createServiceLooseOffer();
    serviceConfirmMutualExclusion = createServiceConfirmMutualExclusion();
    serviceRequestCurrentExecutionPath = createServiceRequestCurrentExecutionPath();
    serviceReceiveReachPoints = createServiceReceiveReachPoints();
    serviceInformReachPoint = createServiceInformReachPoint();
    
    offerManager.startTaskProcessing(); //Task executor, processes accepted tasks one after one


    //Test for mutual exclusion 
    // std::thread trd(testMutual);
    // trd.detach();
   
    SharedFunctions::color.r  = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
    SharedFunctions::color.g  = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
    SharedFunctions::color.b  = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);


    //testRAI();
    
    //TEST FOR KOLLISIONDETECTION
    //Setup publisher for rvis markers
    // ros::Publisher pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    // SharedFunctions::visPub = &pub;

    //TEST COLLISIONPLANNER
    
    // std::thread collisionThread(testCollisionPlanner);
    // collisionThread.detach();
    // END TEST COLLISIONPLANNER
    //Reset marker
    geometry_msgs::Point spoint;
    spoint.x = SharedFunctions::currentPosition.x;
    spoint.y = SharedFunctions::currentPosition.y;
    setMarker(spoint);
    sleep(1);
    setMarker(spoint);

    //Main Loop
    ros::Rate loop_rate(10);
    while( ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

void testCollisionPlanner(){
    CollisionPlanner planner;
    // geometry_msgs::Point p1,p2,p3,p4,p5,p6;
    // p1.x = 351;
    // p1.y = 561;
   
    // p2.x = 364;
    // p2.y = 183;
    
    // p3.x = 767;
    // p3.y = 176;

    // p4.x = 721;
    // p4.y = 560;

    // p5.x = 127;
    // p5.y = 355;

    // p6.x = 919;
    // p6.y = 331;


    // p5.x = 850;
    // p5.y = 500;
    std::vector<geometry_msgs::Point> vectorPoints1;
    std::vector<geometry_msgs::Point> vectorPoints2;

    double curX = -10;
    double  curY = 10;
    for (int i = 0; i < 19;i++){
        geometry_msgs::Point p;
        p.x = curX;
        p.y = curY;

        curX-=1;
        curY+=1;
        vectorPoints1.push_back(p);
    }
    geometry_msgs::Point p;
    p.x = -29;
    p.y = 29;
    vectorPoints1.push_back(p);
    p.y = 30;
    vectorPoints1.push_back(p);
    p.y = 31;
    vectorPoints1.push_back(p);
    p.y = 32;
    vectorPoints1.push_back(p);

    //second
    curX = 20;
    curY = 48;
    for(int i = 0; i < 65; i++){
         geometry_msgs::Point p;
        p.x = curX;
        p.y = curY;

        curX-=1;
        curY-=1;
        vectorPoints2.push_back(p);
    }
    p.x = -44;
    p.y = -17;
    vectorPoints2.push_back(p);
    p.y = -18;
    vectorPoints2.push_back(p);
    p.y = -19;
    vectorPoints2.push_back(p);
    p.y = -20;
    vectorPoints2.push_back(p);
    p.y = -21;
    vectorPoints2.push_back(p);
    

    // vectorPoints1.push_back(p1);
    // vectorPoints1.push_back(p2);
    
    // vectorPoints1.push_back(p3);
    // vectorPoints2.push_back(p4);
    
    // vectorPoints2.push_back(p5);
    // vectorPoints2.push_back(p6);
    // vectorPoints2.push_back(p5);
    CollisionResult result = planner.calculateCollisions(vectorPoints1,vectorPoints2,2,2);
    
    if( result.isValid){
        for( int i = 0; i < result.collisions.size(); i++){
            std::cout << "Wait on: [" << result.collisions[i].waitPoint.x << "," << result.collisions[i].waitPoint.y << "] ";
            std::cout << "Reach to: [" << result.collisions[i].reachPoint.x << "," << result.collisions[i].reachPoint.y << "] " << std::endl;
        }
    }else{
        std::cout << "Invalid result for collisionPlanner" << std::endl;
    }
    
}

void testRAI(){
    RAI rai;
    Cost cost;

    //Set current execution task
    autonomous_robot::TransportTask executionTask;
    geometry_msgs::Pose2D startPose;
        geometry_msgs::Pose2D endPose;
        if ( SharedFunctions::currentExecutionTransportTask.id == -1 ){
            //There is no task, create a new task who does represent the robot 
            startPose.x = SharedFunctions::currentPosition.x;
            startPose.y = SharedFunctions::currentPosition.y;
            endPose.x = SharedFunctions::currentPosition.x;
            endPose.y = SharedFunctions::currentPosition.y;
            executionTask.id = -1; //Represents robot without task
        }else{
            //Only endpoint is needed, the locatoin when the robot has finished his current task.
            startPose.x = SharedFunctions::currentExecutionTransportTask.startPoint.x;
            startPose.y = SharedFunctions::currentExecutionTransportTask.startPoint.y;
            endPose.x = SharedFunctions::currentExecutionTransportTask.endPoint.x;
            endPose.y = SharedFunctions::currentExecutionTransportTask.endPoint.y;
            executionTask.id = SharedFunctions::currentExecutionTransportTask.id;
        }

        executionTask.endPoint = endPose;
        executionTask.startPoint = startPose;

    // std::vector<autonomous_robot::TransportTask> testTasks;
    // testTasks.push_back(task2);
    // testTasks.push_back(task3);
    autonomous_robot::TransportTask task2 = createTestTask(10,10,10,20,2);
    autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);
    autonomous_robot::TransportTask task4 = createTestTask(50,50,60,60,4);
    autonomous_robot::TransportTask task5 = createTestTask(70,70,80,80,5);
    autonomous_robot::TransportTask task6 = createTestTask(90,90,100,100,6);
    // autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);
    // autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);
    // autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);
    // autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);
    // autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);
    // autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);
    // autonomous_robot::TransportTask task3 = createTestTask(30,30,40,40,3);

    // SharedFunctions::acceptedTasks->push_back(task2);
    // SharedFunctions::acceptedTasks->push_back(task3);
    SharedFunctions::acceptedTasks->push_back(task4);
    // SharedFunctions::acceptedTasks->push_back(task5);

    // for ( int i = 0; i < testTasks.size(); i++){
        cost = rai.calculateOrderWithNewTask(task3,task2);
        //Zum testen manuell setzen    
        //SharedFunctions::acceptedTasks->assign(SharedFunctions::calculatedOrderForNewTask->begin(),SharedFunctions::calculatedOrderForNewTask->end());
         std::cout << "END cost " << cost.costDistance << std::endl;
        std::cout << std::endl << std::endl << std::endl;

    // }
    // std::cout << "END cost " << cost << std::endl;

    
    // cost = rai.calculateOrderWithNewTask(task3);
    // std::cout << "END cost " << cost << std::endl;
    //SharedFunctions::calculatedOrderForNewTask->clear();
    // std::vector<autonomous_robot::TransportTask> tasks = rai.calculateOrderWithNewTask(task2);
    // SharedFunctions::calculatedOrderForNewTask->assign(tasks.begin(),tasks.end());
    
    // std::vector<autonomous_robot::TransportTask> tasks = rai.calculateOrderWithNewTask(task3);
    // SharedFunctions::calculatedOrderForNewTask->assign(tasks.begin(),tasks.end());
   

}

autonomous_robot::TransportTask createTestTask(double x1, double y1, double x2, double y2, int id){
    autonomous_robot::TransportTask task;
    geometry_msgs::Pose2D startPoint;
    geometry_msgs::Pose2D endPoint;
    startPoint.x = x1;
    startPoint.y = y1;
    
    endPoint.x = x2;
    endPoint.y = y2;
    task.endPoint = endPoint;
    task.startPoint = startPoint;
    task.id = id;
    return task;
}

void readRobotsFromFile(){
    bool exists = existsFile(FILE_ROBOTS_LIST);
    if ( exists ){
        std::ifstream infile(FILE_ROBOTS_LIST);
        std::string line;
        while ( std::getline(infile,line)){
            if ( ! line.empty() ){
                SharedFunctions::robotNames->push_back(line);
            }
        }   
    }else{
        ROS_ERROR_STREAM("Robots file not found, expected file " << FILE_ROBOTS_LIST);
        exit(EXIT_NO_ROBOT_FILE_FOUND);
    }
}

void readConfig(){
    bool exists = existsFile(FILE_CONFIG);
    if ( exists ){
        std::ifstream infile(FILE_CONFIG);
        std::string line;
        while ( std::getline(infile,line)){
            if ( !line.empty()){
                std::istringstream iss(line);
                std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                                std::istream_iterator<std::string>());
                if( results.size() == 2){
                    if( results[0].compare("distribution_mode") == 0){
                        SharedFunctions::distributionMode = atoi(results[1].c_str());
                    }else if (results[0].compare("simulate_driving") == 0){
                        int value = atoi(results[1].c_str());
                        if( value == 0){
                            SharedFunctions::simulateDrivingForTests = true;
                        }else{
                            SharedFunctions::simulateDrivingForTests = false;
                        }
                    }
                }
            }
        }   
    }else{
        ROS_INFO_STREAM("Config file not found, expected file " << FILE_CONFIG);
    }
}

bool existsFile (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

std::string getFilePath (const std::string& str){
  std::size_t found = str.find_last_of("/\\");
  return str.substr(0,found);
  //str.substr(found+1) would be only file name without path
}

ros::ServiceServer createServiceRequestMutualExclusion(){
     ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService("RequestMutualExclusion",&MutualExclusion::callbackRequestMutualExclusion,&mutualExclusion);   //Only Master node
    ROS_INFO("Service RequestMutualExclusion registered");
    return server;
}
ros::ServiceServer createServiceReleaseMutualExclusion(){
     ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService("ReleaseMutualExclusion",&MutualExclusion::callbackReleaseMutualExclusion,&mutualExclusion);   //Only Master node
    ROS_INFO("Service ReleaseMutualExclusion registered");
    return server;
}
ros::ServiceServer createServiceConfirmMutualExclusion(){
     ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService(SharedFunctions::getServiceName(SharedFunctions::nodeID,"ConfirmMutualExclusion"),&MutualExclusion::callbackConfirmMutualExclusion,&mutualExclusion);
    ROS_INFO("Service ConfirmMutualExclusion registered");
    return server;
}

ros::ServiceServer createServiceLooseOffer(){
    ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService(SharedFunctions::getServiceName(SharedFunctions::nodeID,"LooseOffer"),&OfferManager::callbackLooseOffer,&offerManager);
    ROS_INFO("Service LooseOffer registered");
    return server;
}

ros::ServiceServer createServerWinOffer(){
    ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService(SharedFunctions::getServiceName(SharedFunctions::nodeID,"WinOffer"),&OfferManager::callbackWinOffer,&offerManager);
    ROS_INFO("Service WinOffer registered");
    return server;
}

ros::ServiceServer createServerReceiveCounterOffer(){
    ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService("ReceiveCounterOffer",&TaskDistributer::callbackReceiveCounterOffer,&distributer);
    ROS_INFO("Service ReceiveCounterOffer registered");
    return server;
}

ros::ServiceServer createServerReceiveOffer(){
    ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService(SharedFunctions::getServiceName(SharedFunctions::nodeID,"ReceiveOffer"),&OfferManager::callbackReceiveOffer,&offerManager);
    ROS_INFO("Service ReceiveOffer registered");
    return server;
}


ros::ServiceServer createServerAddNewTransportTask(){
    ros::ServiceServer server;
    server = SharedFunctions::nodeHandler->advertiseService("AddNewTransportTask",&TaskDistributer::callbackAddNewTransportTask,&distributer);
    ROS_INFO("Service AddNewTransportTask registered");
    return server;
}

ros::ServiceServer createServiceRequestCurrentExecutionPath(){
    ros::ServiceServer server;
    std::string serviceName = SharedFunctions::getServiceName(SharedFunctions::nodeID,"RequestCurrentExecutionPath");
    server = SharedFunctions::nodeHandler->advertiseService(serviceName,&OfferManager::callbackRequestCurrentExecutionPath,&offerManager);
    ROS_INFO("Service RequestCurrentExecutionPath registered");
    return server;
}

ros::ServiceServer createServiceReceiveReachPoints(){
    ros::ServiceServer server;
    std::string serviceName = SharedFunctions::getServiceName(SharedFunctions::nodeID,"ReceiveReachPoints");
    server = SharedFunctions::nodeHandler->advertiseService(serviceName,&OfferManager::callbackReceiveReachPoints,&offerManager);
    ROS_INFO("Service ReceiveReachPoints registered");
    return server;
}

ros::ServiceServer createServiceInformReachPoint(){
    ros::ServiceServer server;
    std::string serviceName = SharedFunctions::getServiceName(SharedFunctions::nodeID,"InformReachPoint");
    server = SharedFunctions::nodeHandler->advertiseService(serviceName,&OfferManager::callbackInformReachPoint,&offerManager);
    ROS_INFO("Service InformReachPoint registered");
    return server;
}

void publishPosition(){
   
    autonomous_robot::RobotPosition pos;
    pos.location = SharedFunctions::currentPosition;
    pos.from = SharedFunctions::nodeID;
    pos.radius = SharedFunctions::radius;
    SharedFunctions::position_publisher->publish(pos);
}