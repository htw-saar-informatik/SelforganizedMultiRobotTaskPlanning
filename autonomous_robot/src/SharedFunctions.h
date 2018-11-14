
#ifndef SHAREDFUNCTIONS_H_
#define SHAREDFUNCTIONS_H_

#include <string>
#include <sstream>
#include <condition_variable>
#include <mutex>

#include "autonomous_robot/TransportTask.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

struct Color{
    double r;
    double g;
    double b;
};
class SharedFunctions{
public:
    /**
     * Generates a name for a service in the form nodeID/serviceName
     * @param nodeID name of this node
     * @param serviceName name of the service 
     */
    static std::string getServiceName(std::string nodeID, std::string serviceName);
    static std::string getServiceName(std::string nodeID, const char *serviceName);
    // static double costCurrentTasks; //Represents the current Cost for all tasks for this node, deadheadtime
    // static double costWithNewTask;  //Calculated cost, if this task has additionally the new offered task, dedheadtime
    // static double costCurrentTasksExecutionTime; //Represents the current Cost for all tasks for this node, deadheadtime
    // static double costWithNewTaskExecutionTime;  //Calculated cost, if this task has additionally the new offered task, dedheadtime

    static bool startPositionForTransportTaskReached;   //true, if the robot currently is on the way to the endposition, false if he is on the way to the startpositon
    static std::mutex mutexSetStartPositionReached;
    static std::mutex mutexChangePosition;  //To get access of the current position while the robot is driving
    static geometry_msgs::Pose2D currentPosition;   //Represents the current position of the robot
    static int frame_id;
    static double radius;
    static bool simulateDrivingForTests; //If this is set to true, the task execution gets simulated and not executed. Used for task distribution test withouht driving

    static int distributionMode;    //0 for distance distribution, 1 for parallel execution time distribution

    static int countLinesAreParallel;
    static int countCollisionPathStartsWithClosedWay;
    static int notEveryPathIsClosed;
    //The task who is in execution at the moment, this task cannot be changed until the task execution finishes
    static autonomous_robot::TransportTask currentExecutionTransportTask;
    //This are the tasks this robot won the offers, they are ready to get executet
    //The tasks are in the correct execution order for minimal costs, order can change when a new task has won
    static std::vector<autonomous_robot::TransportTask> *acceptedTasks;
    //This vector holds the order of the tasks resulting from the calculation when a new tasks arives.
    static std::vector<autonomous_robot::TransportTask> *calculatedOrderForNewTask;
    static std::string nodeID;
    static ros::NodeHandle *nodeHandler;
    static std::vector<std::string> *robotNames;
    static std::vector<autonomous_robot::TransportTask> * getAcceptedTransportTasks();
    static std::mutex mutex_mutual_exclusion;   ///mutex and condition variable are used to request mutual exclusion and wait as long as the right is obtained
    static std::condition_variable condition_mutual_exclusion;

    static void printAcceptedTasks();
    static void printcalculatedOrderForNewTask();
    static void printCurrentExecutionTask();
    static void printTask(autonomous_robot::TransportTask task);
    static int getNextId(); //Get next unique id for a transporttask (int counter)
    static ros::Publisher* visPub;  //Publisher for rviz markers
    static Color color;
    static ros::Publisher* position_publisher;

    static void drawLines(std::vector<geometry_msgs::Point> points, int id, bool show);

private:
    SharedFunctions();  //No Objects for this class
    static int idCounter;
};

#endif /* SHAREDFUNCTIONS_H_ */