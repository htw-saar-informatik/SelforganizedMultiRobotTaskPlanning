#include "DriveSimulator.h"

DriveSimulator::DriveSimulator() {
	// TODO Auto-generated constructor stub

}

DriveSimulator::~DriveSimulator() {
	// TODO Auto-generated destructor stub
}


 /**
  * Start simulation for the transport task
  */ 
 void DriveSimulator::simulateTransportTask(autonomous_robot::TransportTask task){

     sendTextForTask(task,true);
    //  sleep(1000);
    sendTextForTask(task,true);
     
    //Drive to the startPoint of the task
    bool drived = false;
    geometry_msgs::Point p;
    p.x = task.startPoint.x;
    p.y = task.startPoint.y;
    //Check if the robot has driven to the position. Its possible that the driving is not possible (e.g a not moving robot stands in his way)
    while( !drived ){
        drived = driveToPosition(p);
        printSimulationData();
        if(!drived){
            ROS_ERROR("Could not drive to position, try it again in 5 seconds");
            sleep(5000);
        }
    }
    SharedFunctions::mutexSetStartPositionReached.lock();
    SharedFunctions::startPositionForTransportTaskReached = true;
    SharedFunctions::mutexSetStartPositionReached.unlock();
    
    p.x = task.endPoint.x;
    p.y = task.endPoint.y;
    drived = false;
    while( !drived ){
        drived = driveToPosition(p);
        printSimulationData();
        if(!drived){
            ROS_ERROR("Could not drive to position, try it again in 5 seconds");
            sleep(5000);
        }
    }
   
    sendTextForTask(task,false);
    //  sleep(1000);
    sendTextForTask(task,false);

    
 }

 void DriveSimulator::printSimulationData(){
    std::cout << "countIntersectionsDetected: " << countIntersectionsDetected << " ";
    std::cout << "countDriveNotPossible: " << countDriveNotPossible << " ";
    std::cout << "countCollisionResultIsInvalid: " << countCollisionResultIsInvalid << " ";
    std::cout << "countRobotHasToWaitForSignal: " << countRobotHasToWaitForSignal << std::endl;
    std::cout << "totalWaitingTimeForSignal: " << totalWaitingTimeForSignal << std::endl;
    
    std::cout << countIntersectionsDetected << "\t" << countDriveNotPossible << "\t" << countCollisionResultIsInvalid << "\t" <<
                countRobotHasToWaitForSignal << "\t" << totalWaitingTimeForSignal << "\t" << std::endl;

    std::cout << "countLinesAreParalell: " << SharedFunctions::countLinesAreParallel  << " ";
    std::cout << "countCollisionPathStartsWithClosedWay: " << SharedFunctions::countCollisionPathStartsWithClosedWay << " ";
    std::cout << "notEveryPathIsClosed: " << SharedFunctions::notEveryPathIsClosed << std::endl;

     std::cout << SharedFunctions::countLinesAreParallel << "\t" << SharedFunctions::countCollisionPathStartsWithClosedWay << "\t" <<
     SharedFunctions::notEveryPathIsClosed << std::endl;
 }

    /**
     * Calculate path for driving, request all current Paths from all robots, check for collision on paths, sends all reachPoints to robots and starts simulation
     * of driving
     */ 
 bool DriveSimulator::driveToPosition(geometry_msgs::Point position){

    //1) Request mutual exclusion for this drive
    bool hasMutualExclusion = requestMutualExclusion();
    if ( hasMutualExclusion ){

        //Here, the path used before can be deleted. The path is only deleted when the robot starts a new tour, for incoming reachPoints if the robot has already reached his goal.
        currentPathIndex = 0;
        waitPoints.clear();
        reachPoints.clear();
        mutexGoalReached.lock();
                goalReached = false;
        mutexGoalReached.unlock();
         //2) Calculate points for the route to the position who get traversed
        geometry_msgs::Point currentPos;
        currentPos.x = SharedFunctions::currentPosition.x;
        currentPos.y = SharedFunctions::currentPosition.y;

        mutexCurrentPath.lock();
            pointsForCurrentPath = calculatePointsForPath(currentPos,position);
        mutexCurrentPath.unlock();

        //3) Get CurrentExecutionPath from all other robots
        std::vector<PathResult> paths = requestPathsFromOtherRobots();
        
        std::vector<SendingReachPoint> sendingReachPoints;  //First collect all points to send to other robots. Only send them, when each robot has a valid collisionPath;
        bool canDrive = true;

        for( int i = 0; i < paths.size();i++){
            std::cout << "Path " << paths[i].from << std::endl;
            std::vector<geometry_msgs::Point> pointsRobotTwo = paths[i].path;

            //Check for collisoin on both paths
            CollisionPlanner planner;
            
            CollisionResult result = planner.calculateCollisions(pointsForCurrentPath,pointsRobotTwo,SharedFunctions::radius,paths[i].radius);
            std::vector<int> reachPointList;
            if( result.isValid){
                if( !result.canDrive){
                    ROS_INFO("There is no possible way to drive, but result is valid");
                    countIntersectionsDetected++;
                    canDrive = false;
                }else{
                    for( int j = 0; j < result.collisions.size(); j++){
                        geometry_msgs::Point waitPoint = result.collisions[j].waitPoint;
                        geometry_msgs::Point reachPoint = result.collisions[j].reachPoint;

                        std::cout << "Wait on: [" << waitPoint.x << "," << waitPoint.y << "] ";
                        std::cout << "Reach to: [" << reachPoint.x << "," << reachPoint.y << "] " << std::endl;

                        //Get Id of the reachPoint
                        int index = -1;
                        for( int k = 0; k < pointsRobotTwo.size();k++){
                            if ( pointsRobotTwo[k].x == reachPoint.x && pointsRobotTwo[k].y == reachPoint.y){
                                index = k;
                            }
                        }
                        if ( index == -1){
                            ROS_ERROR("ReachPoint not found!");
                        }
                        reachPointList.push_back(index);
                        //Save waitPoint
                        WaitPoint wp;
                        wp.waitId = getIdForWaitPoint(waitPoint);
                        wp.robotID = paths[i].from;
                        wp.informId = index;
                        wp.informReceived = false;
                        mutexWaitPoints.lock();
                            waitPoints.push_back(wp);
                        mutexWaitPoints.unlock();
                    
                        //Save this reachPoint
                        SendingReachPoint sendingReachPoint;
                        sendingReachPoint.to = paths[i].from;
                        sendingReachPoint.indices = reachPointList;
                        sendingReachPoints.push_back(sendingReachPoint);
                    }

                    if ( result.collisions.size() == 0){
                        std::cout << "No collisons found" << std::endl;
                    }
                }
                

            }else{
                canDrive = false;
                ROS_ERROR("Invalid result for collisionPlanner, robot can not drive, data:");
                countCollisionResultIsInvalid++;
                std::cout << "Radius: " << SharedFunctions::radius << " " << paths[i].radius << std::endl;
                std::cout << "CurrentPath: " << std::endl;
                for(int i = 0; i < pointsForCurrentPath.size();i++){
                    std::cout << i << ") " << pointsForCurrentPath[i].x << " " << pointsForCurrentPath[i].y << std::endl;;
                }
                std::cout << "pointsRobotTwo: " << std::endl;
                for(int i = 0; i < pointsRobotTwo.size();i++){
                    std::cout << i << ") " << pointsRobotTwo[i].x << " " << pointsRobotTwo[i].y << std::endl;;
                }
                ROS_ERROR("END DATA");
            }

        }

       
        //Check if robot is allowed to drive
        if ( !canDrive ){
            countDriveNotPossible++;
            countIntersectionsDetected += waitPoints.size();
            pointsForCurrentPath.clear();
            waitPoints.clear();
             releaseMutualExclusion();
            return false;
        }else{
             std::cout << "Calculated Waitpoints: " << std::endl;
             printWaitPoints();
            countIntersectionsDetected += waitPoints.size();
            //Send reachPoints to all Robots
            for(int i = 0; i < sendingReachPoints.size();i++){
                sendReachPointsToRobot(sendingReachPoints[i].to,sendingReachPoints[i].indices);
            }

            releaseMutualExclusion();
            simulateDriving();
            mutexGoalReached.lock();
                goalReached = true;
            mutexGoalReached.unlock();
            return true;
        }  

    }else{
        return false;
    }

 }

/**
 * Send the points to the robot, on which it should inform this robot on reaching
 */ 
void DriveSimulator::sendReachPointsToRobot(std::string robotID,std::vector<int> reachPoints){
        ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<autonomous_robot::ReceiveReachPoints>(SharedFunctions::getServiceName(robotID,"ReceiveReachPoints"));
        autonomous_robot::ReceiveReachPoints srv;
        srv.request.from = SharedFunctions::nodeID;
        // srv.request.pointIndices = reachPoints;
        for(int i = 0; i < reachPoints.size(); i++){
             srv.request.pointIndices.push_back(reachPoints[i]);
        }

        if (client.call(srv)){
            ROS_INFO_STREAM("ReachPoints send to Robot " << robotID);    
        }else{
            ROS_ERROR_STREAM("Failed to call service ReceiveReachPoints for " << robotID);
        }
}

/**
 * Send a message to a robot that this robot has reached a point he is waiting for
 */ 
void DriveSimulator::informRobotAboutReachedPoint(ReachPoint point){
    ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<autonomous_robot::InformReachPoint>(SharedFunctions::getServiceName(point.to,"InformReachPoint"));
        autonomous_robot::InformReachPoint srv;
        srv.request.from = SharedFunctions::nodeID;
        srv.request.index = point.index;

        if (client.call(srv)){
            ROS_INFO_STREAM("InformReachpoint send to Robot " << point.to);    
        }else{
            ROS_ERROR_STREAM("Failed to call service InformReachPoint for " << point.to);
        }
}

/**
 *  Easier to send only the index of the points instead of the actual point
 */ 
 int DriveSimulator::getIdForWaitPoint(geometry_msgs::Point p){
     for(int i = 0; i < pointsForCurrentPath.size();i++){
         //Points should be exactly equal
         if ( pointsForCurrentPath[i].x == p.x && pointsForCurrentPath[i].y == p.y){
             return i;
         }
     }
     return -1;
 }

/**
 * Send a message to each robot(not to the robot itself) and collect their current paths in execution
 */ 
 std::vector<PathResult> DriveSimulator::requestPathsFromOtherRobots(){
    std::vector< PathResult > paths;

    for( int i = 0; i < SharedFunctions::robotNames->size();i++){
        std::string name = SharedFunctions::robotNames->at(i);
        if( name.compare(SharedFunctions::nodeID) != 0 ){
            ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<autonomous_robot::RequestCurrentExecutionPath>(SharedFunctions::getServiceName(name,"RequestCurrentExecutionPath"));
            //std_srvs::Empty empty;
            autonomous_robot::RequestCurrentExecutionPath srv;

            if (client.call(srv)){
                PathResult result;
                result.path = srv.response.path;
                result.radius = srv.response.radius;
                result.from = srv.response.from;
                paths.push_back(result);
                ROS_INFO_STREAM("RequestCurrentExecutionPath for " << name);    
            }else{
                ROS_ERROR_STREAM("Failed to call service RequestCurrentExecutionPath for " << name);
            }
        }
    }
    std::cout << "[requestPathsFromOtherRobos] Got all paths from all robots:";
    for( int i = 0; i < paths.size();i++){
        std::cout << "Path from " << paths[i].from << "Radius: " << paths[i].radius << std::endl;
        std::vector<geometry_msgs::Point>& vec = paths[i].path;
        for( int j = 0; j < vec.size(); j++){
            geometry_msgs::Point p = vec[j];
            std::cout << "x: " << p.x << " y: " << p.y << std::endl;
        }
    }
    return paths;
 }

 /**
 * Calculates points on the path between startPoint and endPoint, so that a robot can traverse this points on his path
 */ 
std::vector<geometry_msgs::Point> DriveSimulator::calculatePointsForPath(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint){
    
    bool finishedY = false;
    bool finishedX = false;
    double value = 1;
    double epsilon = value;
     std::vector<geometry_msgs::Point> vec;

     //Add startpoint to vec
    vec.push_back(startPoint);
    while( !(finishedY && finishedX) ){
        //Check x pos
        double difx = fabs(startPoint.x-endPoint.x);
        double dify = fabs(startPoint.y-endPoint.y);
       
        // std::cout  << " X [" << startPoint.x << " " << endPoint.x << " " << difx << "] ";
        // std::cout  << "Y [" << startPoint.y << " " << endPoint.y << " " << dify << "] " << std::endl;
        if ( difx <= epsilon ){
            //Add rest
            if(startPoint.x < endPoint.x){
                startPoint.x+=difx;
            }else if (startPoint.x > endPoint.x){
                startPoint.x-=difx;
            }
            finishedX = true;
        }else{
            if(startPoint.x < endPoint.x){
                startPoint.x+=value;
            }else if (startPoint.x > endPoint.x){
                startPoint.x-=value;
            }
        }

        if ( dify <= epsilon ){
             if(startPoint.y < endPoint.y){
                startPoint.y+=dify;
            }else if (startPoint.y > endPoint.y){
                startPoint.y-=dify;
            }
            finishedY = true;
        }else{
            if(startPoint.y < endPoint.y){
                startPoint.y+=value;
            }else if (startPoint.y > endPoint.y){
                startPoint.y-=value;
            }
        }
        vec.push_back(startPoint);
    }

    // for( int i = 0; i < vec.size();i++){
    //     std::cout << vec[i].x << "," << vec[i].y << std::endl;
    // }
    return vec;
}

/**
 * Changes the current position of the robot until the robot reaches the goal position.
 * The robot can move every round value in x and y direction. It stops, when the goal in both directions is reached
 * 
 */ 
void DriveSimulator::simulateDriving(){
    
    //Draw the new line
    drawPath(true);
    // sleep(1000);
    drawPath(true);

    for( int i = 0; i < pointsForCurrentPath.size();i++){
        //Move to new position
        geometry_msgs::Point p = pointsForCurrentPath[i];
        //Set global position
        SharedFunctions::mutexChangePosition.lock();
            SharedFunctions::currentPosition.x = p.x;
            SharedFunctions::currentPosition.y = p.y;
            currentPathIndex = i;
            //Set marker on rviz
            setMarker(p);
            sendTransform(p);
            std::cout << "Drived to x: " << p.x  << " y: " << p.y <<  std::endl;
            //Inform about position change:
            publishPosition();
        SharedFunctions::mutexChangePosition.unlock();
        //Check here after changing the position if the robot has to inform other robots about reaching this position
        mutexReachPoints.lock();
            for(int i = 0; i < reachPoints.size();i++){
                if ( reachPoints[i].index == currentPathIndex ){
                    std::cout << "I have to inform robot " << reachPoints[i].to  << std::endl;
                    // sleep(1000*5);
                    informRobotAboutReachedPoint(reachPoints[i]);
                }
            }
        mutexReachPoints.unlock();
        //Check herer, if the robot has to wait on this position, or if the robot has to notify another robot
        bool startSet = false;
        auto start = std::chrono::high_resolution_clock::now();
        while(checkIfRobotHasToWait()){
            if (!startSet){
                start = std::chrono::high_resolution_clock::now();
                startSet = true;
            }
            std::cout << "I ( " << SharedFunctions::nodeID << " ) have to wait here for receiving a reach point" << std::endl;
            //wait here until the robot receives a new reachInform, then check again (Its possible that he has to wait on this position for more than one reachPoint)
            std::unique_lock<std::mutex> lock(mutexWaitOnPoint);
            while(!notifiedWaitOnPoint){
                condition_waitOnPoint.wait(lock);
            }
            notifiedWaitOnPoint = false;
        }
        if(startSet){
            countRobotHasToWaitForSignal++;
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            totalWaitingTimeForSignal += elapsed.count();
        }
        //Wait before the robot drives to next position
        sleep(DRIVING_SLEEP_MILLIS);
    }

    //Delete Line
    std::vector<geometry_msgs::Point> empty;
    drawPath(false);
    SharedFunctions::drawLines(empty,SharedFunctions::frame_id*100+3,false);
    SharedFunctions::drawLines(empty,SharedFunctions::frame_id*100+4,false);
    // sleep(1000);
    drawPath(false);
    SharedFunctions::drawLines(empty,SharedFunctions::frame_id*100+3,false);
    SharedFunctions::drawLines(empty,SharedFunctions::frame_id*100+4,false);
    

}

 bool DriveSimulator::checkIfRobotHasToWait(){
    bool result = false;
    mutexWaitPoints.lock();
        for( int i = 0; i < waitPoints.size(); i++){
            if( waitPoints[i].waitId == currentPathIndex){
                //Robot has to wait here
                if( waitPoints[i].informReceived == false){
                    result = true;
                }
            }
        }
    mutexWaitPoints.unlock();
    return result;
 }

/**
 *  The robot received a reachpoint, search for corresponding waitpoint and mark it as received
 */ 
 void DriveSimulator::informReachPoint(std::string from, int index){
     std::cout << "[informReachPoint] Robot " << from << " informed me about reaching his point with index " << index << std::endl; 
     mutexWaitPoints.lock();
        //Search the waitPoint to this reachpoint and mark it as received;
        for(int i = 0; i < waitPoints.size();i++){

            if( waitPoints[i].robotID.compare(from) == 0){
                if( waitPoints[i].informId == index){
                    waitPoints[i].informReceived = true;
                    std::cout << "Successfully set this reachpoint as received" << std::endl;
                }
            }
        }

        //Inform the robot that a new reachPoint was received
        notifiedWaitOnPoint = true;
        condition_waitOnPoint.notify_one();
     mutexWaitPoints.unlock();
 }

/**
 * Has to be called in a mutex of changePosition
 */ 
 void DriveSimulator::addReachPoint(ReachPoint point){
     reachPoints.push_back(point);
 }




//Sets a marker in rviz
void DriveSimulator::setMarker(geometry_msgs::Point p){
    static ros::Publisher vis_pub = SharedFunctions::nodeHandler->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = SharedFunctions::frame_id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

     //size of CYLINDER
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
    // marker.color.r = 0.5;
    // marker.color.g = 0.5;
    // marker.color.b = 0.5;

    marker.color.r = SharedFunctions::color.r;
    marker.color.g = SharedFunctions::color.g;
    marker.color.b = SharedFunctions::color.b;

    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();


    // std::cout << "Publishing " << std::endl;
    vis_pub.publish( marker );
}


void DriveSimulator::sleep(int millis){
     std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

bool DriveSimulator::requestMutualExclusion(){
    bool hasRight = false;
    ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<autonomous_robot::RequestMutualExclusion>("RequestMutualExclusion");
    autonomous_robot::RequestMutualExclusion requestMutualExclusionService;
    requestMutualExclusionService.request.nodeID = SharedFunctions::nodeID;

    if (client.call(requestMutualExclusionService)){
        ROS_INFO("Call for requestMutualExclison successful");
        hasRight = true;
    }
    else
    {
        ROS_ERROR("Failed to call service RequestMutualExclusion to master");
        return false;
    }

    //Warte hier, bis mutual exclusion erhalten wurde
    std::cout << "Wait here until the right for mutual exclusion is obtained "  << SharedFunctions::nodeID << std::endl;
    std::unique_lock<std::mutex> lock(SharedFunctions::mutex_mutual_exclusion);
    SharedFunctions::condition_mutual_exclusion.wait(lock);
    std::cout << "I have obtained the right to do a mutual exclusion "  << SharedFunctions::nodeID  << std::endl;
    return true;
}

void DriveSimulator::releaseMutualExclusion(){
    ROS_INFO("release mutual exclusion");
    MutualExclusion::sendReleaseMessage();
}

void DriveSimulator::printWaitPoints(){
    mutexWaitPoints.lock();
    for(int i = 0; i < waitPoints.size();i++){
        std::cout << "Wait on Position " << waitPoints[i].waitId << ", wait until robot " << waitPoints[i].robotID << "reaches position " << waitPoints[i].informId << " , isInformed= " << waitPoints[i].informReceived <<  std::endl;
    }
    mutexWaitPoints.unlock();
}

void DriveSimulator::printReachPoints(){
    mutexReachPoints.lock();
    for( int i = 0; i < reachPoints.size();i++){
        std::cout << "Reach position " << reachPoints[i].index << ", inform robot " << reachPoints[i].to << std::endl;
    }
    mutexReachPoints.unlock();
}

void DriveSimulator::drawPath(bool show){
    SharedFunctions::drawLines(pointsForCurrentPath,SharedFunctions::frame_id*100,show);
}



void DriveSimulator::sendTextForTask(autonomous_robot::TransportTask task, bool show){
    static ros::Publisher vis_text_pub = SharedFunctions::nodeHandler->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.lifetime = ros::Duration();
    marker.id = SharedFunctions::frame_id*100+1;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    if(show){
        marker.action = visualization_msgs::Marker::ADD;
    }else{
        marker.action = visualization_msgs::Marker::DELETE;
    }
    
    marker.pose.position.x = task.startPoint.x;
    marker.pose.position.y = task.startPoint.y;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    std::stringstream ss;
    ss << SharedFunctions::frame_id << "S";
    marker.text = ss.str();

    marker.scale.x = 10;
    marker.scale.y = 10;
    marker.scale.z = 4;

    // marker.color.r = 1.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.color.a = 1.0;
    marker.color.r = SharedFunctions::color.r;
     marker.color.g = SharedFunctions::color.g;
    marker.color.b = SharedFunctions::color.b;
    marker.color.a = 1.0;
    vis_text_pub.publish( marker );

    //Send endpoint

    marker.id = SharedFunctions::frame_id*100+2;
    marker.pose.position.x = task.endPoint.x;
    marker.pose.position.y = task.endPoint.y;
    std::stringstream ss2;
    ss2 << SharedFunctions::frame_id << "E";
     marker.text = ss2.str();
     vis_text_pub.publish( marker );

}

/**
 * Call this function in a positionmutex
 */ 
void DriveSimulator::publishPosition(){
    // static ros::Publisher position_publisher = SharedFunctions::nodeHandler->advertise<autonomous_robot::RobotPosition>("RobotPositionChange",1000);
    autonomous_robot::RobotPosition pos;
    pos.location = SharedFunctions::currentPosition;
    pos.from = SharedFunctions::nodeID;
    pos.radius = SharedFunctions::radius;
    SharedFunctions::position_publisher->publish(pos);
}

/**
 * Sends a transform so that other nodes can use this information
 */ 
void DriveSimulator::sendTransform(geometry_msgs::Point p ){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = SharedFunctions::nodeID;
    transformStamped.transform.translation.x = p.x;
    transformStamped.transform.translation.y = p.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0,1);
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    br.sendTransform(transformStamped);
}