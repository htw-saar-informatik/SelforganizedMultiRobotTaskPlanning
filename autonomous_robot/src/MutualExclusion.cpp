#include "MutualExclusion.h"


/**
 * Gets called on the master when a node wants mutual Exclusion in the system
 */ 
 bool MutualExclusion::callbackRequestMutualExclusion(autonomous_robot::RequestMutualExclusion::Request& request, autonomous_robot::RequestMutualExclusion::Response& response){

     mutex_mutualExclusion.lock();
        ROS_INFO("[MutualExclusion] New request for mutual exclusion arrived:");
        
        waitingNodes.push(request.nodeID);
        //Check if node can be confirmed
        notified = true;
        condition_mutualExclusion.notify_one();
    mutex_mutualExclusion.unlock();
    return true;
 }

/**
 * Starts an thread who processes existing requests
 */ 
 void MutualExclusion::processRequests(void){
    std::thread trd(&MutualExclusion::runTask,this); 
    trd.detach();

}

/**
 *Start a thread that runs till the process ends.
 *Thread waits till a request for a mutual exclusion is available. 
 *if there is no more task, the thread waits again for notification
 * 
 */
void MutualExclusion::runTask(){
     ROS_INFO("[MutualExclusion] Thread mutual exclusion started");
        while(true){    //runs till the process is terminated
            std::unique_lock<std::mutex> lock1(mutex_mutualExclusion);    //This lock is for condition waiting
            while(!notified){
                ROS_INFO("[MutualExclusion] No requests available, wait for notify ");
                condition_mutualExclusion.wait(lock1); //wait till a new request is available
            }
            lock1.unlock();
            bool leave = false;
            do{
                
                std::string name;
                //take next request
                std::unique_lock<std::mutex> lock2(mutex_mutualExclusion);    //this lock is for processing requests until all requests are processed
                //mutex_tasks.lock();
                    if( waitingNodes.empty() ){
                        ROS_INFO("[MutualExclusion] No requests left, wait again to get notified");
                        leave = true;
                        notified = false;
                        lock2.unlock();
                    }else{
                        name = waitingNodes.front();
                        waitingNodes.pop();
                        lock2.unlock();
                        sendConfirmMessage(name);
                        ROS_INFO("[MutualExclusion] task processing finished ");
                    }
            }while(!leave);
        }
}


/**
 * Sends an confirm message to a node and waits until the node releases his mutual exclusion again
 */ 
void MutualExclusion::sendConfirmMessage(std::string nodeID){
    // sleep(2);

    ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<std_srvs::Empty>(SharedFunctions::getServiceName(nodeID,"ConfirmMutualExclusion"));
    std_srvs::Empty empty;


    if (client.call(empty)){
        ROS_INFO_STREAM("[MutualExclusion] Confirm mutual exclusion for  " << nodeID);    
    }else{
        ROS_ERROR_STREAM("[MutualExclusion] Failed to call service ConfirmMutualExclusion for " << nodeID);
    }

    //wait until the robots releases the mutual exclusion again
    std::unique_lock<std::mutex> lock(mutex_mutualExclusionRelease);
    condition_mutualExclusionRelease.wait(lock);
}


/**
 * Gets called when this robots gets an ok to go in the critical section
 */
bool MutualExclusion::callbackConfirmMutualExclusion(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    //here the client is allowed to enter the critical section
    ROS_INFO("[MutualExclusion] confirm received");
    
    //Inform about this
    SharedFunctions::condition_mutual_exclusion.notify_one();
    return true;
}

void MutualExclusion::sendReleaseMessage(){
    ros::ServiceClient client = SharedFunctions::nodeHandler->serviceClient<std_srvs::Empty>("ReleaseMutualExclusion");
    std_srvs::Empty empty;


    if (client.call(empty)){
        ROS_INFO_STREAM("[MutualExclusion] Confirm mutual exclusion to master");    
    }else{
        ROS_ERROR_STREAM("[MutualExclusion] Failed to call service releaseMutualExclusion to master ");
    }

}

/**
 * Gets called on the master when a robots releases his mutual exclusion
 */ 
 bool MutualExclusion::callbackReleaseMutualExclusion(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
      mutex_mutualExclusionRelease.lock(); 
        ROS_INFO("[MutualExclusion] Received release for mutual exclusion ");
            condition_mutualExclusionRelease.notify_one();
    mutex_mutualExclusionRelease.unlock();
    return true;
 }


MutualExclusion::MutualExclusion() {
	// TODO Auto-generated constructor stub

}

MutualExclusion::~MutualExclusion() {
	// TODO Auto-generated destructor stub
}

