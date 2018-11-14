#ifndef OFFERMANAGER_H_
#define OFFERMANAGER_H_

#include "ros/ros.h"
#include "autonomous_robot/ReceiveOffer.h"
#include "autonomous_robot/ReceiveCounterOffer.h"
#include "autonomous_robot/RequestCurrentExecutionPath.h"
#include "autonomous_robot/ReceiveReachPoints.h"
#include "autonomous_robot/InformReachPoint.h"
#include "std_srvs/Empty.h"
#include "SharedFunctions.h"
#include "RAI.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "DriveSimulator.h"

/**
 * Combines task offer handling and task execution in this class, can be seperated later
 */ 
class OfferManager {
public:
	OfferManager();
	virtual ~OfferManager();
    bool callbackReceiveOffer(autonomous_robot::ReceiveOffer::Request &req,
                                autonomous_robot::ReceiveOffer::Response &res);
    bool callbackWinOffer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool callbackLooseOffer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool callbackReceiveReachPoints(autonomous_robot::ReceiveReachPoints::Request &reg, autonomous_robot::ReceiveReachPoints::Response &res);
    bool callbackInformReachPoint( autonomous_robot::InformReachPoint::Request &req, autonomous_robot::InformReachPoint::Response &res );
    void startTaskProcessing();

    //Callbacks for task execution
    bool callbackRequestCurrentExecutionPath(autonomous_robot::RequestCurrentExecutionPath::Request& request, autonomous_robot::RequestCurrentExecutionPath::Response &res);
private:
    DriveSimulator driveSimulator;
    void processOfferThread(autonomous_robot::TransportTask task,std::string from, std::string to);
    bool isWaitingForOfferResult;
    bool isNotified;
    std::mutex mutex_execution;
    std::condition_variable condition_execution;
    void processTasks();    //Processes the tasks this node has won
    void executeTask(); //Is responsible for executing a task

    int drivedDistance = 0;
    std::mutex mutexDrivedDistance;
    int executedTime= 0;
   
    void sleepMillis(int milliseconds);

    std::vector<autonomous_robot::TransportTask> executedTasks;
};

#endif /* OFFERMANAGER_H_ */
