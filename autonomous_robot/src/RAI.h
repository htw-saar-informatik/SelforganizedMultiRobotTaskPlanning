#ifndef RAI_H_
#define RAI_H_

#include <autonomous_robot/TransportTask.h>
#include "SharedFunctions.h"
#include <sstream>
#include <vector>
#include <stdlib.h> //abs
#include <math.h>   //pow
#include <time.h>
#include <random>

struct Cost{
    double costDistance;
    double costExecutionTime;
};

class RAI {
public:
	RAI();
	virtual ~RAI();

    Cost calculateOrderWithNewTask(autonomous_robot::TransportTask task, autonomous_robot::TransportTask currentExecutionTask);
    static double calculateCostForTour(std::vector<autonomous_robot::TransportTask>& s);
    static double calculateCostForTourExecutionTime(std::vector<autonomous_robot::TransportTask>& s);
private:
   std::vector< std::vector<double> > costMatrix;
    void createCostMatrix();
    double calculateCostForTasks(autonomous_robot::TransportTask task1, autonomous_robot::TransportTask task2);
    autonomous_robot::TransportTask newTask;    //Represents the task who gets added
    std::vector<autonomous_robot::TransportTask> availableTasks;    //Store all tasks for matrix index
    std::vector<autonomous_robot::TransportTask> calculateTour();
    int getRandomBetween(int min, int max);
    autonomous_robot::TransportTask& getVertexNotOnTour(std::vector<autonomous_robot::TransportTask>& s);
    std::vector<autonomous_robot::TransportTask> insertCheap(std::vector<autonomous_robot::TransportTask>& s,autonomous_robot::TransportTask nextTask);
   
    double getCostForTour(std::vector<autonomous_robot::TransportTask>& s);
    double getCostForTourExecutionTime(std::vector<autonomous_robot::TransportTask>& s);

    int getIndexForTask(autonomous_robot::TransportTask task);
    void printTour(std::vector<autonomous_robot::TransportTask>& s);
    std::vector<autonomous_robot::TransportTask> optimizeTour(std::vector<autonomous_robot::TransportTask> originalTour);
    void changeOrder(std::vector<autonomous_robot::TransportTask>& tour);
    autonomous_robot::TransportTask currentExecutionTask;  //This is either the current execution task or the robot itself
    void log(std::string function);
    bool showLogs = false;
    std::stringstream logStream;
};

#endif /* RAI_H_ */
