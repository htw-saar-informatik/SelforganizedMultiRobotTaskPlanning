#include "SharedFunctions.h"

//initialize static variables, otherwise they are not accessable
std::string SharedFunctions::nodeID = std::string("");
ros::NodeHandle *SharedFunctions::nodeHandler = nullptr;
autonomous_robot::TransportTask SharedFunctions::currentExecutionTransportTask;
std::vector<autonomous_robot::TransportTask> * SharedFunctions::acceptedTasks = nullptr;
std::vector<std::string> * SharedFunctions::robotNames = nullptr;
std::vector<autonomous_robot::TransportTask> * SharedFunctions::calculatedOrderForNewTask = nullptr;
std::mutex SharedFunctions::mutex_mutual_exclusion;
std::condition_variable SharedFunctions::condition_mutual_exclusion;
int SharedFunctions::idCounter = 0;
bool SharedFunctions::startPositionForTransportTaskReached = false;
std::mutex SharedFunctions::mutexSetStartPositionReached;
std::mutex SharedFunctions::mutexChangePosition;
int SharedFunctions::distributionMode = 0;

int SharedFunctions::countLinesAreParallel = 0;
int SharedFunctions::countCollisionPathStartsWithClosedWay = 0;
int SharedFunctions::notEveryPathIsClosed = 0;
// double SharedFunctions::costCurrentTasks = 0;
// double SharedFunctions::costWithNewTask = 0; 
// double SharedFunctions::costCurrentTasksExecutionTime = 0;
// double SharedFunctions::costWithNewTaskExecutionTime = 0;  

geometry_msgs::Pose2D SharedFunctions::currentPosition;
ros::Publisher* SharedFunctions::visPub = nullptr;
int SharedFunctions::frame_id = -1;
double SharedFunctions::radius = 2;
Color SharedFunctions::color;
ros::Publisher* SharedFunctions::position_publisher = nullptr;
bool SharedFunctions::simulateDrivingForTests = false;

std::string SharedFunctions::getServiceName(std::string nodeID, std::string serviceName){
        std::stringstream sstr;
        sstr << nodeID << "/" << serviceName;
        return sstr.str();
}
std::string SharedFunctions::getServiceName(std::string nodeID, const char *serviceName){
    return getServiceName(nodeID,std::string(serviceName));
}


std::vector<autonomous_robot::TransportTask> * SharedFunctions::getAcceptedTransportTasks(){
    return acceptedTasks;
}

int SharedFunctions::getNextId(){
    return idCounter++;
}

void SharedFunctions::printAcceptedTasks(){
    for( int i = 0; i < acceptedTasks->size();i++){
        printTask(acceptedTasks->at(i));
    }
}
void SharedFunctions::printcalculatedOrderForNewTask(){
    for( int i = 0; i < calculatedOrderForNewTask->size();i++){
        printTask(calculatedOrderForNewTask->at(i));
    }
}
void SharedFunctions::printCurrentExecutionTask(){
    printTask(currentExecutionTransportTask);
}

void SharedFunctions::printTask(autonomous_robot::TransportTask task){
     std::cout << "Task{ id: " << task.id << " , start: {x: " << task.startPoint.x << ", y: " << task.startPoint.y << "}, ";
    std::cout << " end: {x: " << task.endPoint.x << ", y:" << task.endPoint.y << "}}" << std::endl;
}


void SharedFunctions::drawLines(std::vector<geometry_msgs::Point> points, int id, bool show){
    static ros::Publisher vis_line_pub = SharedFunctions::nodeHandler->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    //std::cout << "Draw a line now, id " << id <<std::endl;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
    if (show){
        marker.action = visualization_msgs::Marker::ADD;     
    }else{
        marker.action = visualization_msgs::Marker::DELETE;
    }
	marker.lifetime = ros::Duration();
	marker.id = id;	//Has to be unique for every line
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	
	// marker.pose.position.x = 1;
	// marker.pose.position.y = 1;
	// marker.pose.position.z = 1;
	// marker.pose.orientation.x = 0.0;
	// marker.pose.orientation.y = 0.0;
	// marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.3;
	// marker.scale.y = 0.1;
	// marker.scale.z = 0.1;
	
    marker.color.a = 1.0; // Don't forget to set the alpha!
	// marker.color.r = 0.0;
	// marker.color.g = 1.0;
	// marker.color.b = 0.0;

    marker.color.r = SharedFunctions::color.r;
    marker.color.g = SharedFunctions::color.g;
    marker.color.b = SharedFunctions::color.b;

	marker.points = points;
    vis_line_pub.publish(marker);
}