/*
 * TaskDistributer.cpp
 *
 *  Created on: 24.09.2018
 *      Author: markus
 */

#include "TaskDistributer.h"

// void TaskDistributer::printTask(autonomous_robot::TransportTask task){
//     std::cout << "Task start [" << task.startPoint.x << "," << task.startPoint.y << "] ";
//     std::cout << " end [" << task.endPoint.x << "," << task.endPoint.y << "]" << std::endl;
// }


void TaskDistributer::processTasks(void){
    std::thread trd(&TaskDistributer::runTask,this); 
    trd.detach();
}

bool TaskDistributer::callbackReceiveCounterOffer(autonomous_robot::ReceiveCounterOffer::Request &req,
                                autonomous_robot::ReceiveCounterOffer::Response &res){

    mutex_counterOffer.lock(); 
        std::cout << "[TaskDistributer] Counteroffer Received: " << req.from << " " << req.costWithoutTask << " " << req.costWithTask << std::endl;
        offerResults.push_back(req);
        // notified = true;

        //Check if all robots have send Counteroffer
        int availableRobots = SharedFunctions::robotNames->size();
        int errorRobotsCount = errorRobots.size();
        int difference = availableRobots - errorRobotsCount;
        if ( offerResults.size() == difference){
                notifiedCounterOffer = true;
                condition_counterOffer.notify_one();
                std::cout << "[callbackReceiveCounterOffer] Offer for all robots received" << std::endl;
        }
    mutex_counterOffer.unlock();

    //no success response required
    return true;
}

bool TaskDistributer::callbackAddNewTransportTask(autonomous_robot::AddNewTransportTask::Request &req,
                                autonomous_robot::AddNewTransportTask::Response &res){
    // std::cout << "Received task:" << std::endl;
    // distributer.printTask(req.task);
     //std::unique_lock<std::mutex> lock(mutex_tasks);
    mutex_tasks.lock(); 
        ROS_INFO("[TaskDistributer] New Task arrived:");
        req.task.id = SharedFunctions::getNextId();
        SharedFunctions::printTask(req.task);
        transportTasks.push(req.task);
        notified = true;
        condition_tasks.notify_one();
        std::cout << "[TaskDistributer] Task sucessfully added " << std::endl;
    mutex_tasks.unlock();
    //Check if task can be processed (only task available)
    //Without mutex is no problem because if size is 0, there is no other running thread
    
    res.success = 0;
    return true;
}


void TaskDistributer::startTaskProcessing(){
    processTasks();
}

/**
 *Start a thread that runs till the process ends.
 *Thread waits till a transporttask is available, then it processes this task (in a new thread) one after one
 *if there is no more task, the thread waits again for notification
 * 
 */
void TaskDistributer::runTask(){
       std::cout << "[TaskDistributer] Thread TaskDistributer started" << std::endl;
        while(true){    //runs till process is terminated
            //std::unique_lock<std::mutex> lock(mutex_tasks);
            std::unique_lock<std::mutex> lock1(mutex_tasks);    //This lock is for condition waiting
            while(!notified){
                std::cout << "[TaskDistributer] No tasks available, wait till i get notified " << std::endl;
                condition_tasks.wait(lock1); //wait till task is available
            }
            lock1.unlock();
            bool leave = false;
            do{
                autonomous_robot::TransportTask task;
                //take next task
                std::unique_lock<std::mutex> lock2(mutex_tasks);    //this lock is for processing tasks until all available tasks are processed
                //mutex_tasks.lock();
                    if( transportTasks.empty() ){
                        std::cout << "[TaskDistributer] No tasks left, wait again to get notified" << std::endl;
                        leave = true;
                        notified = false;
                        //mutex_tasks.unlock();
                        lock2.unlock();
                    }else{
                        task = transportTasks.front();
                        transportTasks.pop();
                        //mutex_tasks.unlock();
                        lock2.unlock();
                        std::cout << "[TaskDistributer] task found, process task: " << std::endl;
                        SharedFunctions::printTask(task);
                        sendOfferToAllRobots(task);
                        std::cout << "[TaskDistributer] task processing finished " << std::endl;
                    }
            }while(!leave);
        }
}

void TaskDistributer::sendOfferToAllRobots(autonomous_robot::TransportTask task){
    offerResults.clear();
    errorRobots.clear();

    // double time1=0.0, tstart;
    // tstart = clock();  //Beginn time tracking
    auto start = std::chrono::high_resolution_clock::now();
    notifiedCounterOffer = false;

    //Send offer to every robot one after one (call is very fast so no threads for parallel execution needed (to make it simpler, could change if call needs more time on robot side)
    for( std::string name: *SharedFunctions::robotNames){
        
        ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<autonomous_robot::ReceiveOffer>(SharedFunctions::getServiceName(name,"/ReceiveOffer"));
        autonomous_robot::ReceiveOffer receiveOfferService;
        receiveOfferService.request.task = task;
        receiveOfferService.request.from = SharedFunctions::nodeID;
        receiveOfferService.request.to = name;


        if (client.call(receiveOfferService))
        {
            std::int8_t success = receiveOfferService.response.success;
            //std_msgs::Int8 success = addNewTransportTaskService.response.success;
            
            if( success == 0){
                ROS_INFO_STREAM("[TaskDistributer] Task send to " << name);    
            }else{
                ROS_INFO("[TaskDistributer] There was an error : %d",success);
                errorRobots.push_back(name);
            }
        }
        else
        {
            errorRobots.push_back(name);
            ROS_ERROR_STREAM("[TaskDistributer] Failed to call service ReceiveOffer for " << name);
        }
    }

    //Await answer from every Robot
    std::cout << "[TaskDistributer] Offer send to all Robots, wait till i get notified " << std::endl;
    std::unique_lock<std::mutex> lock(mutex_counterOffer);
    while( !notifiedCounterOffer){
        condition_counterOffer.wait(lock);
    }
    lock.unlock();

    std::cout << "[TaskDistributer] Received counterOffer from all robots, counterOffers:" << std::endl;
    //Look which robot has made the best counteroffer, send him a win message
    
    //distributionMode 0 = distance, 1 = parallel execution time
     float minCost = -1;
    autonomous_robot::ReceiveCounterOffer::Request * winnerRequest = nullptr;
    if ( SharedFunctions::distributionMode == 0){
        for ( autonomous_robot::ReceiveCounterOffer::Request& req : offerResults){
            std::cout << req.from << " DeadHeadDistance: " << req.costWithoutTask  << " -> " << req.costWithTask;
            std::cout << " ExecutionTime: " << req.costWithoutTaskExecutionTime << " -> " << req.costWithTaskExecutionTime << " totalAproxExecutionTime: " << req.approximatedExecutionTime << std::endl;
            if( minCost == -1 ){
                minCost = (req.costWithTask - req.costWithoutTask);
                winnerRequest = &req;
            }else{
                float tmpCost = (req.costWithTask - req.costWithoutTask);
                if (  tmpCost < minCost ){
                    minCost = tmpCost;
                    winnerRequest = &req;
                }
            }
        }
    }else{
            //Parallel execution time, assign task to the robot who has the lowest execution time of all robots
         for ( autonomous_robot::ReceiveCounterOffer::Request& req : offerResults){
            std::cout << req.from << " DeadHeadDistance: " << req.costWithoutTask  << " -> " << req.costWithTask;
            std::cout << " ExecutionTime: " << req.costWithoutTaskExecutionTime << " -> " << req.costWithTaskExecutionTime << " totalAproxExecutionTime: " << req.approximatedExecutionTime << std::endl;
            
            if( minCost == -1 ){
                minCost = req.costWithTaskExecutionTime;
                winnerRequest = &req;
            }else{
                if (  req.costWithTaskExecutionTime < minCost ){
                    minCost = req.costWithTaskExecutionTime;
                    winnerRequest = &req;
                }
            }
        }
    }

    totalCostDeadheadDistance += (winnerRequest->costWithTask-winnerRequest->costWithoutTask);
    // time1 += clock() - tstart;     // end timetracking
    // time1 = time1/CLOCKS_PER_SEC;  //in sekunden umwandeln
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    totalCalculationTime += elapsed.count();
   

    std::cout << "[TaskDistributer] Winner is: " << winnerRequest->from << std::endl;

    //Save the aproxExecutionTime of the winner
    executionTimeForRobots[winnerRequest->from] = winnerRequest->approximatedExecutionTime;
    //List all elements
   

    //Inform winner and send him a message
    ros::ServiceClient clientNew = SharedFunctions::nodeHandler->serviceClient<std_srvs::Empty>(SharedFunctions::getServiceName(winnerRequest->from,"/WinOffer"));
    std_srvs::Empty empty;
    if (!clientNew.call(empty)){
        ROS_ERROR_STREAM("[TaskDistributer] Failed to call service WinOffer");
    }else{
        ROS_INFO("[TaskDistributer] WinOffer successfully called");
    }

    std::cout << "[TaskDistributer] Inform all other clients about loosing" << std::endl;
    for (std::string name: *SharedFunctions::robotNames){
        if ( name.compare(winnerRequest->from) != 0){
            ros::ServiceClient clientLooseOffer = SharedFunctions::nodeHandler->serviceClient<std_srvs::Empty>(SharedFunctions::getServiceName(name,"/LooseOffer"));
            std_srvs::Empty empty;

            if (!clientLooseOffer.call(empty)){
                ROS_ERROR_STREAM("[TaskDistributer] Failed to call service LooseOffer");
            }else{
                //ROS_INFO("WinOffer successfull called");
            }
            }
        }

    //TransportTask was send to an Robot, return and process next request
    std::cout << std::endl << "------------------------" << std::endl;
    std::cout << "[TaskDistributer] " << taskNr << ") totalDeadheadDistance: " << totalCostDeadheadDistance << " totalCalculationTime: " << totalCalculationTime << " s" << std::endl;
    std::cout << "All executionTimes for robots: " << std::endl;
    double longestExecutionTime = 0;
    for(auto it = executionTimeForRobots.cbegin(); it != executionTimeForRobots.cend(); ++it){
        std::cout << it->first << " " << it->second << std::endl;
        if( it->second > longestExecutionTime){
            longestExecutionTime = it->second;
        }
    }
    std::cout << "TotalExecutionTime: " << longestExecutionTime << std::endl;
    std::cout << "------------------------" << std::endl << std::endl;
    taskNr++;
}



void TaskDistributer::sleep(int seconds){
     std::this_thread::sleep_for(std::chrono::seconds(seconds));
}



TaskDistributer::TaskDistributer() {
    //Start processing Task, runs as long as this process runs
   //processTasks();
}

TaskDistributer::~TaskDistributer() {
	// TODO Auto-generated destructor stub
}

