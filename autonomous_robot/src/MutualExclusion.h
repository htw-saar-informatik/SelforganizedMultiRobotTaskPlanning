#ifndef MUTUALEXCLUSION_H_
#define MUTUALEXCLUSION_H_

#include <queue>
#include <string>
#include "std_srvs/Empty.h"
#include "ros/ros.h"
#include <mutex>
#include <thread>
#include "autonomous_robot/RequestMutualExclusion.h"
#include "SharedFunctions.h"
#include <condition_variable>

class MutualExclusion {
public:
	MutualExclusion();
	virtual ~MutualExclusion();

    bool callbackRequestMutualExclusion(autonomous_robot::RequestMutualExclusion::Request& request, autonomous_robot::RequestMutualExclusion::Response& response);
    bool callbackConfirmMutualExclusion(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool callbackReleaseMutualExclusion(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    static void sendReleaseMessage();
    void processRequests(void);
    

private:
    std::queue<std::string> waitingNodes;
    
    std::mutex mutex_mutualExclusion;
    std::condition_variable condition_mutualExclusion;

    std::mutex mutex_mutualExclusionRelease;
    std::condition_variable condition_mutualExclusionRelease;
    void sendConfirmMessage(std::string nodeID);
    
    void runTask();
    bool notified = true;

    
};

#endif /* MUTUALEXCLUSION_H_ */
