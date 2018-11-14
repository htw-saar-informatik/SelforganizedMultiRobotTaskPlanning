/*
 * TaskDistributer.h
 *
 *  Created on: 24.09.2018
 *      Author: markus
 */

#ifndef TASKDISTRIBUTER_H_
#define TASKDISTRIBUTER_H_

#include <string>
#include <iostream>
#include <autonomous_robot/TransportTask.h>
#include "autonomous_robot/ReceiveOffer.h"
#include "autonomous_robot/ReceiveCounterOffer.h"
#include "autonomous_robot/AddNewTransportTask.h"
// #include "autonomous_robot/WinOffer.h"
#include "std_srvs/Empty.h" //Is used to make service calls who require no parameters
#include <queue>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "ros/ros.h"
#include "SharedFunctions.h"
#include <time.h>

class TaskDistributer {
public:
	TaskDistributer();
	virtual ~TaskDistributer();
    //void printTask(autonomous_robot::TransportTask task);
    void startTaskProcessing();
    // void addRobotNames(std::vector<std::string> *vector);
    // void setNodeHandle(ros::NodeHandle *n);
    bool callbackReceiveCounterOffer(autonomous_robot::ReceiveCounterOffer::Request &req,
                                autonomous_robot::ReceiveCounterOffer::Response &res);
    bool callbackAddNewTransportTask(autonomous_robot::AddNewTransportTask::Request &req,
                                autonomous_robot::AddNewTransportTask::Response &res);
    
private:
    //queue to save all incoming transport requests. Transportrequests will be proccessed one after one
    std::queue<autonomous_robot::TransportTask> transportTasks;
     //Saves all counterOffer results from every node
    std::vector<autonomous_robot::ReceiveCounterOffer::Request> offerResults;
    //Its possible that an Robot does not receive Offer, save which Robots had an error
    std::vector<std::string> errorRobots;
    void processTasks(void);
    void runTask();
    void sendOfferToAllRobots(autonomous_robot::TransportTask task);
    
    //AddNewTask mutex
    std::mutex mutex_tasks;
    std::condition_variable condition_tasks;
    
    //CounterOffer mutex
    std::mutex mutex_counterOffer;
    std::condition_variable condition_counterOffer;
    bool notifiedCounterOffer = false;
    int taskNr = 0;
    void sleep(int seconds);
    bool notified = false;

     std::map<std::string,double> executionTimeForRobots;

    double totalCostDeadheadDistance = 0; //This represents the total amout of deadhead distance for all robots in the system
    double totalCalculationTime = 0.0;      //The time it needs to 1) send all offers to the robots, 2) receive all counterOffers, 3) Calculate the winner of the offer
};

#endif /* TASKDISTRIBUTER_H_ */
