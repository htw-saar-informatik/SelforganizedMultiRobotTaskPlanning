
#ifndef DRIVESIMULATOR_H_
#define DRIVESIMULATOR_H_


#include <autonomous_robot/TransportTask.h>
#include "SharedFunctions.h"
#include "ros/ros.h"
#include "MutualExclusion.h"
#include <geometry_msgs/Point.h>
#include <math.h>   //fabs
#include <cmath> //round
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include "autonomous_robot/RequestCurrentExecutionPath.h"
#include "autonomous_robot/ReceiveReachPoints.h"
#include "autonomous_robot/InformReachPoint.h"
#include "CollisionPlanner/CollisionPlanner.h"
#include <mutex>
#include <condition_variable>
#include <sstream>
#include "autonomous_robot/RobotPosition.h"
#include <thread>
#include <chrono>

struct PathResult{
    double radius;
    std::vector<geometry_msgs::Point> path;
    std::string from;
};

struct SendingReachPoint{
    std::string to;
    std::vector<int> indices;
};

struct WaitPoint{
    int waitId;             //Robot waits on this id
    int informId;           //Other robot has to reach this id
    std::string robotID;    //Name of other robot
    bool informReceived;    //if true, this robot can drive when he arrives at this point
};

//A Robot stores a list of this points, when he reaches the index on his path he informs robot about reaching this point
struct ReachPoint{
    std::string to;
    int index;
};

class DriveSimulator {
public:
    const int DRIVING_SLEEP_MILLIS = 100;
	DriveSimulator();
    void simulateTransportTask(autonomous_robot::TransportTask task);
	virtual ~DriveSimulator();

    void addReachPoint(ReachPoint point);
    void informRobotAboutReachedPoint(ReachPoint point);
    void informReachPoint(std::string from, int index);

    std::vector<geometry_msgs::Point> pointsForCurrentPath;    //This are the points the robot is currently driving, exchange this points with other robots for collision check
    int currentPathIndex;  //To decide for an informationMessage if the robot has reached this point or not, save what the current index in the positons for the task is
    
    std::mutex mutexCurrentPath;    //For all operations on currentPath
   
    std::mutex mutexReachPoints;    //Access the reachPoints on this robot(receive new reachpoint vs go through all reachPoints and inform the robots)
    std::mutex mutexWaitPoints;     //Acess the waitPoints (receive an inform about a reachpoint beeing reached vs check if the robot has to wait)
    std::mutex mutexGoalReached;
    int countIntersectionsDetected;
    int countDriveNotPossible;
    int countCollisionResultIsInvalid;
    int countRobotHasToWaitForSignal;
    double totalWaitingTimeForSignal = 0;
    double distanceToStartPos = 0;
    double distnaceForCurrentTask = 0;

    bool goalReached = false;   //Determines if the robot has reached his goal position. If so, other robots do not need to consider the current path of the robot, wo is stored until the next transport task gets executed.
    //For debugging
    void printWaitPoints();
    void printReachPoints();
private:
    bool driveToPosition(geometry_msgs::Point position);    //True, if it can drive, false if driving is not possible (invalid collision Paths)
    void simulateDriving();
    std::vector<geometry_msgs::Point> calculatePointsForPath(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint);
    // geometry_msgs::Point currentPosition;   //The current position of the robot while it is driving
    void setMarker(geometry_msgs::Point p);
    void sleep(int millis);
    bool requestMutualExclusion();
    void releaseMutualExclusion();
    std::vector<PathResult> requestPathsFromOtherRobots();

    std::vector<WaitPoint> waitPoints;
    std::vector<ReachPoint> reachPoints;
    int getIdForWaitPoint(geometry_msgs::Point point);
    void sendReachPointsToRobot(std::string robotID,std::vector<int> reachPoints);
    bool checkIfRobotHasToWait();

    std::condition_variable condition_waitOnPoint; //Used to wait on a point until a reachPoint inform is received
    bool notifiedWaitOnPoint = false;
    std::mutex mutexWaitOnPoint;
    void drawPath(bool show);
    
    void sendTextForTask(autonomous_robot::TransportTask task, bool show);
    void publishPosition();

    void printSimulationData();
    void sendTransform(geometry_msgs::Point p);
    
};

#endif /* DRIVESIMULATOR_H_ */
