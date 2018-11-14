/*
 * This class is responsible for accepting Offers, calculate the cost for the tasks and send an counteroffer back
 *
 * 
 */

#include "OfferManager.h"


/**
 * Gets called when this robot wins an offer
 */ 
bool OfferManager::callbackWinOffer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    ROS_INFO("[OfferManager] won the offer");
     mutex_execution.lock(); 
        
        //change list with transport tasks
        //delete old order and replace it with the new order
        SharedFunctions::acceptedTasks->assign(SharedFunctions::calculatedOrderForNewTask->begin(),SharedFunctions::calculatedOrderForNewTask->end());
        SharedFunctions::calculatedOrderForNewTask->clear();

        std::cout << "[OfferManager] All current tasks:" << std::endl;
        SharedFunctions::printAcceptedTasks();

        //Set new costs
        // SharedFunctions::costCurrentTasks = SharedFunctions::costWithNewTask;
        // SharedFunctions::costCurrentTasksExecutionTime = SharedFunctions::costWithNewTaskExecutionTime;
        // SharedFunctions::costWithNewTask = 0;
        // SharedFunctions::costWithNewTaskExecutionTime = 0;
        // std::cout << "New: " << std::endl;
        // std::cout << "acceptedTasks: " << std::endl;
        // SharedFunctions::printAcceptedTasks();
        // std::cout << "calculatedTasks " << std::endl;
        // SharedFunctions::printcalculatedOrderForNewTask();
    

        isWaitingForOfferResult = false;

        isNotified = true;
        condition_execution.notify_one();
    mutex_execution.unlock();

    return true;
}

/**
 * task executor, waits till new tasks are won and starts executing them
 */ 
void OfferManager::processTasks(){
    ROS_INFO("[TaskExecutor] Thread task executer started");
    while(true){    //Runs till the process is terminated

        //Wait until there is a new task to process
         std::unique_lock<std::mutex> lock1(mutex_execution);    //This lock is for condition waiting
        while(!isNotified){
            std::cout << "[TaskExecutor] No TransportTasks for execution, wait till i get notified" << std::endl;
            condition_execution.wait(lock1); //wait till task is available
        }

        //lock1.unlock();
        std::cout << "[TaskExecutor] accepted task size: " << SharedFunctions::acceptedTasks->size() << std::endl;

        //Check if the node is waiting for an offerResult
        if ( (!isWaitingForOfferResult) && (SharedFunctions::acceptedTasks->size() != 0) ){
            //Get next task to process, this ensures that a task gets executed after winning, so that not another receiveOffer call is faster
            // std::cout << "accepted tasks vorher: " << std::endl;
            // SharedFunctions::printAcceptedTasks();
            
            std::cout << "[TaskExecutor] Take next task for processing: " << std::endl;
            SharedFunctions::currentExecutionTransportTask = SharedFunctions::acceptedTasks->front();
            SharedFunctions::printCurrentExecutionTask();
            SharedFunctions::acceptedTasks->erase(SharedFunctions::acceptedTasks->begin()); //Delete first element

            lock1.unlock();
            
            //execute task, new tasks can arrive while this task is executed
            executeTask();
            std::cout << "[TaskExecutor] task execution finished, check if there is another task for execution " << std::endl;

            //Check if there are further tasks who can be processed
            bool leave = false;
            //Process tasks as long as there are tasks and the robot is not waiting for an offerResult
            while(!leave){
                lock1.lock();
                if ( isWaitingForOfferResult){
                    std::cout << "[TaskExecutor] waiting for an offer result, don't process" << std::endl;
                }
                if ( isWaitingForOfferResult || SharedFunctions::acceptedTasks->size() == 0){
                    std::cout << "[TaskExecutor] No more tasks, go back to wait" << std::endl;
                    leave = true;
                    isNotified = false;
                    lock1.unlock();
                }else{
                        std::cout << "[TaskExecutor] another task found, process it" << std::endl;
                        //Process next task
                        SharedFunctions::currentExecutionTransportTask = SharedFunctions::acceptedTasks->front();
                        SharedFunctions::acceptedTasks->erase(SharedFunctions::acceptedTasks->begin()); //Delete first element
                        lock1.unlock();
                        executeTask();
                }
            }
            

        }else{
            isNotified = false;
        }
    }
}

/**
 * Execution of this task, this means first there is an collision planner and after that the robot can drive. Task is executed, when to robot is located at the endPosition for this task
 */ 
void OfferManager::executeTask(){
    std::cout << "[TaskExecutor] Processing task ....." << std::endl;
    SharedFunctions::printCurrentExecutionTask();
    //Access shared execution task without mutual exclusin as long as the access is read only
    double drivingTime;
    
    // sleep(10000000);

    //Simulation is to test the distribution of the tasks, not really executing them. is used to calculate costs for dynamic distribution against static distribution
    if ( SharedFunctions::simulateDrivingForTests) {
             //Calculate time to execute this task
            geometry_msgs::Pose2D& position = SharedFunctions::currentPosition;
            autonomous_robot::TransportTask task = SharedFunctions::currentExecutionTransportTask;

            
            position = SharedFunctions::currentPosition;
           

            double costDriveToStartPositon = abs(position.x-task.startPoint.x) + abs(position.y-task.startPoint.y);
            double costDriveFromStartToEndpoint = abs(task.startPoint.x - task.endPoint.x) + abs(task.startPoint.y-task.endPoint.y);
            drivingTime = costDriveToStartPositon + costDriveFromStartToEndpoint;
            
            int speed_driving_time = 100;   //100 milliseconds for one unit

            std::cout << "[TaskExecutor] Time for processing: " << drivingTime << " s" << std::endl;
             SharedFunctions::mutexSetStartPositionReached.lock();
            SharedFunctions::startPositionForTransportTaskReached = false;
            SharedFunctions::mutexSetStartPositionReached.unlock();
            std::cout << "Drive to startPosition, distance: " << costDriveToStartPositon << " drivetime: " << costDriveToStartPositon*speed_driving_time << " ms" << std::endl;
            sleepMillis(costDriveToStartPositon*speed_driving_time);
             SharedFunctions::mutexSetStartPositionReached.lock();
            SharedFunctions::startPositionForTransportTaskReached = true;
            SharedFunctions::mutexSetStartPositionReached.unlock();
            std::cout << "Drive from startPosition to endPosition, distance " << costDriveFromStartToEndpoint << " drivetime: " << costDriveFromStartToEndpoint*speed_driving_time << " ms" << std::endl;
            sleepMillis(costDriveFromStartToEndpoint*speed_driving_time);

            
            //Simulate driving time
        // sleep(drivingTime);
        // sleep(100);
    }else{
         SharedFunctions::mutexSetStartPositionReached.lock();
        SharedFunctions::startPositionForTransportTaskReached = false;
        SharedFunctions::mutexSetStartPositionReached.unlock();

        driveSimulator.simulateTransportTask( SharedFunctions::currentExecutionTransportTask);
    }

    //delete currentTask
   mutex_execution.lock();
        if ( SharedFunctions::simulateDrivingForTests) {
            //In the other case the driveSimulator is responsible for setting the current positon
             SharedFunctions::mutexChangePosition.lock();  
                SharedFunctions::currentPosition = SharedFunctions::currentExecutionTransportTask.endPoint;
            SharedFunctions::mutexChangePosition.unlock();
            SharedFunctions::currentPosition = SharedFunctions::currentExecutionTransportTask.endPoint;

             mutexDrivedDistance.lock();
                // drivedDistance += drivedDistance;
                executedTime += drivingTime;
                std::cout << std::endl << std::endl << std::endl;
                std::cout << "Executed Time: " << executedTime << std::endl;
                std::cout << std::endl << std::endl << std::endl;
            mutexDrivedDistance.unlock();
        }
        //Mark current task as none by setting id to -1
        executedTasks.push_back(SharedFunctions::currentExecutionTransportTask);

        //TEST PRINT ALL TASKS
        std::cout << "ALL TASKS FOR THIS ROBOT " << std::endl;
        for(int i = 0; i < executedTasks.size(); i++){
            std::cout << executedTasks[i].id << std::endl;
        }
        std::cout << "ALL PENDING TASKS " << std::endl;
        for(int i = 0; i < SharedFunctions::acceptedTasks->size();i++){
            std::cout << SharedFunctions::acceptedTasks->at(i).id << std::endl;
        }

        SharedFunctions::currentExecutionTransportTask.id = -1; 
        SharedFunctions::mutexSetStartPositionReached.lock();
        SharedFunctions::startPositionForTransportTaskReached = false;
        SharedFunctions::mutexSetStartPositionReached.unlock();
        
       

        std::cout << "[TaskExecutor] Processing for task finished" << std::endl;
        std::cout << "[TaskExecutor] Current positon of robot: [" << SharedFunctions::currentPosition.x << "," << SharedFunctions::currentPosition.y << "]" << std::endl;
        //Calculate rest cost for tour
        // std::vector<autonomous_robot::TransportTask> tasks = *SharedFunctions::acceptedTasks;
        // SharedFunctions::costCurrentTasks = RAI::calculateCostForTour(tasks);
        // std::cout << "New cost for tour: " << SharedFunctions::costCurrentTasks << std::endl;
        // std::vector<autonomous_robot::TransportTask> tmpTour = *SharedFunctions::acceptedTasks;
        // autonomous_robot::TransportTask startTask;
        // startTask.startPoint = SharedFunctions::currentPosition;
        // startTask.endPoint = SharedFunctions::currentPosition;
        // tmpTour.insert(tmpTour.begin(),startTask);
        // SharedFunctions::costCurrentTasksExecutionTime = RAI::calculateCostForTourExecutionTime(tmpTour);
   mutex_execution.unlock();
   }

/**
 * Gets called when this robot looses an offer
 */ 
bool OfferManager::callbackLooseOffer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    ROS_INFO("[OfferManager] Lost the offer ");
     mutex_execution.lock(); 
        isWaitingForOfferResult = false;
        isNotified = true;
        SharedFunctions::calculatedOrderForNewTask->clear();    //Optionally, should work without
        // SharedFunctions::costWithNewTask = 0;
        condition_execution.notify_one();
    mutex_execution.unlock();
    return true;
}

/**
 * Gets called when this node receives an Offer from Master Node.
 * Starts an background thread who calculates the cost for this offer
 */
bool OfferManager::callbackReceiveOffer(autonomous_robot::ReceiveOffer::Request &req,
                                autonomous_robot::ReceiveOffer::Response &res){
    std::cout << "[OfferManager] Received task:" << req.from << "-> " << req.to <<  " , process cost in background" << std::endl;
    //printTask(req.task);
    mutex_execution.lock();
        isWaitingForOfferResult = true;
    mutex_execution.unlock();
    //Set thread who calculates costs for this task and sends a request back
    std::thread trd(&OfferManager::processOfferThread,this,req.task,req.from,req.to);
    trd.detach();   //Can run in background

    res.success = 0;
    return true;
}



/**
 * This Threads runs in background and calculates the cost for an offer. After calculating, an counteroffer is send back to the master
 * The calculated order (order for tasks with best costs) ist stored in calculatedOrderForNewTask vector. If this node wins the offer, he changes the old 
 * acceptedTasks to this new order
 */ 
void OfferManager::processOfferThread(autonomous_robot::TransportTask task,std::string from, std::string to){
    std::cout << "[OfferManager] Calculate Cost for transport task" << std::endl;
    //std::this_thread::sleep_for(std::chrono::seconds(20));
    double costDistanceWithoutNewTask;
    double costExecutionTimeWithoutNewTask;
    geometry_msgs::Pose2D position;
    RAI rai;
    autonomous_robot::TransportTask executionTask;
    //Check if robot is currently execution a task. If so, this task has to be the first point for the execution
    //Lock is needed because if the current execution task finishes (parallel thread), the pointer for the task is set to nullptr
    mutex_execution.lock();
        SharedFunctions::mutexChangePosition.lock();

            bool startPosReached;
            SharedFunctions::mutexSetStartPositionReached.lock();
                startPosReached = SharedFunctions::startPositionForTransportTaskReached;
            SharedFunctions::mutexSetStartPositionReached.unlock();

            geometry_msgs::Pose2D startPose;
            geometry_msgs::Pose2D endPose;
            position = SharedFunctions::currentPosition;
            if ( SharedFunctions::currentExecutionTransportTask.id == -1 ){
                //There is no task, create a new task who does represent the robot 
                startPose.x = SharedFunctions::currentPosition.x;
                startPose.y = SharedFunctions::currentPosition.y;
                endPose.x = SharedFunctions::currentPosition.x;
                endPose.y = SharedFunctions::currentPosition.y;
                executionTask.id = -1; //Represents robot without task
            }else{
                //the location when the robot has finished his current task.
                startPose.x = SharedFunctions::currentExecutionTransportTask.startPoint.x;
                startPose.y = SharedFunctions::currentExecutionTransportTask.startPoint.y;
                endPose.x = SharedFunctions::currentExecutionTransportTask.endPoint.x;
                endPose.y = SharedFunctions::currentExecutionTransportTask.endPoint.y;
                executionTask.id = SharedFunctions::currentExecutionTransportTask.id;
            }
        SharedFunctions::mutexChangePosition.unlock();
        executionTask.endPoint = endPose;
        executionTask.startPoint = startPose;

        //Calculate cost without new task
        std::vector<autonomous_robot::TransportTask> tour = *SharedFunctions::acceptedTasks;
        //Insert startpostion
        tour.insert(tour.begin(),executionTask);
        costDistanceWithoutNewTask = RAI::calculateCostForTour(tour);   //Deadheadtime for all tasks
        //Check if robot has reached his current task, if not add the deadheaddistance to the current task startpos from current pos
        //  if( !startPosReached ){
        //     if( executionTask.id != -1){
        //         costDistanceWithoutNewTask += abs(position.x-executionTask.startPoint.x) + abs(position.y-executionTask.startPoint.y);
        //     }
        // }



        costExecutionTimeWithoutNewTask = RAI::calculateCostForTourExecutionTime(tour);
        //Chek if the robot has not reached the startPositon of the current execution task, if so add the distance from currentPosition  
        if( !startPosReached ){
            if( executionTask.id != -1){
                costExecutionTimeWithoutNewTask += abs(position.x-executionTask.startPoint.x) + abs(position.y-executionTask.startPoint.y);
            }
        }
    
        
    mutex_execution.unlock();


    Cost cost = rai.calculateOrderWithNewTask(task,executionTask);

    int aproxTime;
    mutexDrivedDistance.lock();
                // aproxDistance = drivedDistance + cost.costDistance;     //Drived deadheaddistance of this robot
                //total execution time up to now for all past tasks + the tasks for pending tasks. executedTime gets increased when this robot finishes a task, so add here this time + the time for the current task inkl. time to drive to startpos
                aproxTime = executedTime + cost.costExecutionTime + abs(position.x-executionTask.startPoint.x) + abs(position.y-executionTask.startPoint.y);
    mutexDrivedDistance.unlock();

    //check if the startpos for currentTask is reached, if not add the distance to first task
     if( !startPosReached ){
            if( executionTask.id != -1){
                // cost.costDistance += abs(position.x-executionTask.startPoint.x) + abs(position.y-executionTask.startPoint.y);
                cost.costExecutionTime += abs(position.x-executionTask.startPoint.x) + abs(position.y-executionTask.startPoint.y);
            }
    }

    // int aproxDistance;
  

    //Send answer back to master node
    ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<autonomous_robot::ReceiveCounterOffer>("ReceiveCounterOffer");
    autonomous_robot::ReceiveCounterOffer receiveCounterOfferService;
    receiveCounterOfferService.request.from = SharedFunctions::nodeID;
    receiveCounterOfferService.request.to = to;
    receiveCounterOfferService.request.task = task;
    receiveCounterOfferService.request.costWithoutTask = costDistanceWithoutNewTask;
    receiveCounterOfferService.request.costWithoutTaskExecutionTime = costExecutionTimeWithoutNewTask;
    // receiveCounterOfferService.request.approximatedDistance = aproxDistance;
    receiveCounterOfferService.request.approximatedExecutionTime = aproxTime;
    // receiveCounterOfferService.request.costWithTask =  SharedFunctions::costCurrentTasks + SharedFunctions::costWithNewTask;
    receiveCounterOfferService.request.costWithTask =  cost.costDistance;

    // receiveCounterOfferService.request.costWithoutTaskExecutionTime = SharedFunctions::costCurrentTasksExecutionTime;
    // receiveCounterOfferService.request.costWithTaskExecutionTime = SharedFunctions::costCurrentTasksExecutionTime + SharedFunctions::costWithNewTaskExecutionTime;
    receiveCounterOfferService.request.costWithTaskExecutionTime = cost.costExecutionTime;
   
    if (client.call(receiveCounterOfferService))
    {
        //No response from master robot
        ROS_INFO("[OfferManager] CounterOffer send to Robot1");    
    }
    else
    {
        ROS_ERROR("[OfferManager] Failed to call service ReceiveOffer");
    }

}

void OfferManager::startTaskProcessing(){
    std::thread trd(&OfferManager::processTasks,this); 
    trd.detach();
}


/**
 *  Gets called when another robot wants the current execution path from this robot
 */ 
bool OfferManager::callbackRequestCurrentExecutionPath(autonomous_robot::RequestCurrentExecutionPath::Request& request, autonomous_robot::RequestCurrentExecutionPath::Response &res){
    ROS_INFO("[DriveSimulator] Got a request for my current execution path");
    driveSimulator.mutexCurrentPath.lock();

        //Check if this robot has reached his goal already
        bool goalReached;
        driveSimulator.mutexGoalReached.lock();
                    goalReached = driveSimulator.goalReached;
                    std::cout << "Goal reached: " << goalReached << std::endl;
        driveSimulator.mutexGoalReached.unlock();

        std::vector<geometry_msgs::Point> points;
        if( goalReached ){
            //Just return current Position, not the path.
            geometry_msgs::Point p;
            p.x = SharedFunctions::currentPosition.x;
            p.y = SharedFunctions::currentPosition.y;
            points.push_back(p);
        }else{
             points = driveSimulator.pointsForCurrentPath;
            if ( points.size() == 0){
                //Robot has no points, add its current position
                geometry_msgs::Point p;
                p.x = SharedFunctions::currentPosition.x;
                p.y = SharedFunctions::currentPosition.y;
                points.push_back(p);
                // p.x +=0.1;
                // p.y +=0.1;
                // points.push_back(p); 
            }
        }
       
        res.path = points;
        res.from = SharedFunctions::nodeID;
        res.radius = SharedFunctions::radius;
    driveSimulator.mutexCurrentPath.unlock();
    return true;
}

bool OfferManager::callbackInformReachPoint( autonomous_robot::InformReachPoint::Request &req, autonomous_robot::InformReachPoint::Response &res ){
    driveSimulator.informReachPoint(req.from,req.index);
    return true;
}

bool OfferManager::callbackReceiveReachPoints(autonomous_robot::ReceiveReachPoints::Request &reg, autonomous_robot::ReceiveReachPoints::Response &res){

    std::cout << "[DriveSimulator] Received Reach Points from: " << reg.from << std::endl;
    std::cout << "[DriveSimulator] Current ReachPoints: " << std::endl;
    driveSimulator.printReachPoints();

    for ( int i = 0; i < reg.pointIndices.size(); i++){
        std::cout << "Index: " << reg.pointIndices[i] << std::endl;
        //Check if this robot has already traversed this point
        int currentIndex;
        SharedFunctions::mutexChangePosition.lock();
            currentIndex = driveSimulator.currentPathIndex;
            std::cout << "[DriveSimulator] Current Index: " << currentIndex << std::endl;
            ReachPoint reachPoint;
            reachPoint.index = reg.pointIndices[i];
            reachPoint.to = reg.from;
            if( reg.pointIndices[i] <= currentIndex){
                std::cout << "[DriveSimulator] Point already reached, send directly back an inform" << std::endl;
                //Robot can directly be informed
                driveSimulator.informRobotAboutReachedPoint(reachPoint);
            }else{
                std::cout << "[DriveSimulator] Add new reachpoint to list " << std::endl;
                driveSimulator.mutexReachPoints.lock();
                    driveSimulator.addReachPoint(reachPoint);
                driveSimulator.mutexReachPoints.unlock();
            }
        SharedFunctions::mutexChangePosition.unlock();

    }
    return true;
}

void OfferManager::sleepMillis(int milliseconds){
     std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

OfferManager::OfferManager() {
	// TODO Auto-generated constructor stu


}

OfferManager::~OfferManager() {
	// TODO Auto-generated destructor stub
}

