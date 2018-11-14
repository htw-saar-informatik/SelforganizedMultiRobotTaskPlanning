#include "RAI.h"

/**
 * Calculated the order for all available tasks depending on the new task and sets the global vector calculatedOrderForNewTask with the new order
 * currentExecutionTask = task who is currently in execution and therefore the order can not be changed, nullptr if there is no such task
 */
Cost RAI::calculateOrderWithNewTask(autonomous_robot::TransportTask task, autonomous_robot::TransportTask currentExecutionTask){
    double time1=0.0, tstart;
    tstart = clock();  //Beginn time tracking
    double costDistance = 0;
    double costExecutionTime = 0;
    newTask = task;
    this->currentExecutionTask = currentExecutionTask; 
    //#1 create costDistance matrix for all tasks who already exist + the new task
    createCostMatrix();

    //#2 calculate the new order for the tour with lowest costDistance
    std::vector<autonomous_robot::TransportTask> tour = calculateTour();
    
    costDistance = getCostForTour(tour);
    //Has to be called before changeOrder
    costExecutionTime = getCostForTourExecutionTime(tour);

    //#3 the tour hast to be changed so that it starts with currentExecution Task/ Robot task 
    std::cout << "Tour before changeOrder" << std::endl;
    // logStream << "Tour before changeOrder";
    // log("calculateOrderWithNewTask");
    printTour(tour);
    changeOrder(tour);
    std::cout << "Tour after changeOrder " << std::endl;
    // logStream << "Tour after changeOrder";
    // log("calculateOrderWithNewTask");
    printTour(tour);
    //SharedFunctions::calculatedOrderForNewTask->assign(tasks.begin(),tasks.end());
    
    //#4 Set the calculated tour
    SharedFunctions::calculatedOrderForNewTask->assign(tour.begin(),tour.end());
    std::cout << "New Calculated Order: " << std::endl;
    SharedFunctions::printcalculatedOrderForNewTask();
    std::cout << "Cost for tour distance: " << costDistance << std::endl;
    std::cout << "Cost for tour execution time: " << costExecutionTime << std::endl;
     time1 += clock() - tstart;     // end timetracking
     time1 = time1/CLOCKS_PER_SEC;  //in sekunden umwandeln
    std::cout << "  time = " << time1 << " sec." << std::endl;
    // SharedFunctions::costWithNewTask = costDistance;
    // SharedFunctions::costWithNewTaskExecutionTime = costExecutionTime;

    Cost cost;
    cost.costDistance = costDistance;
    cost.costExecutionTime = costExecutionTime;
    return cost;
}

/**
 * initializes the costmatrix, fills the available tasks
 */ 
void RAI::createCostMatrix(){
    int size = SharedFunctions::acceptedTasks->size()+2;    //+2, one for the new task, one for the task in execution/the robot itself if there is no execution task

    costMatrix.resize(size, std::vector<double>(size, 0));

    availableTasks.assign(SharedFunctions::acceptedTasks->begin(),SharedFunctions::acceptedTasks->end());

   
    //Add the new Task
    availableTasks.push_back(newTask);

    //Add current execution task
    availableTasks.push_back(currentExecutionTask);

      //List all available tasks
    std::cout << "All available tasks:" << std::endl;
    for(int i = 0; i < availableTasks.size(); i++){
        SharedFunctions::printTask(availableTasks[i]);
    }

    //Calculate costs for each element
    // std::cout << "Cost Matrix: " << std::endl;
    for( int i = 0; i < size; i++){
		for ( int j = 0; j < size; j++){
			costMatrix[i][j] = calculateCostForTasks(availableTasks[i],availableTasks[j]);
            // std::cout << costMatrix[i][j] << ", ";
		}
		// std::cout << std::endl;
	}

}

/**
 *  Calculates the time for execution for two tasks one after another.
 */ 
double RAI::calculateCostForTasks(autonomous_robot::TransportTask task1, autonomous_robot::TransportTask task2){
    //Its possible to use different strategies, now use manhatten distance

    if( task1.id == task2.id){
        //If the tasks are equal, calculate the time to execute the task (task1->startPoint to task1->endPoint)
        return abs(task1.startPoint.x - task1.endPoint.x) + abs(task1.startPoint.y-task1.endPoint.y);
    }else{
        bool task2IsStartTask = false;  //true, if task2 is robot position or position for current Execution task
        task2IsStartTask = (task2.id == currentExecutionTask.id);
        if ( task2IsStartTask ){
            //Task -> Robot|currentExecutionTask, cost is 0 because this means that task1 is the last task on the tour
            return 0;
        }else{
            //Task -> Task && Robot-> Task, calculate the time from (task1|Robot)->endPoint to task2->startPoint
            return abs(task1.endPoint.x-task2.startPoint.x) + abs(task1.endPoint.y-task2.startPoint.y);
        }

    }

}

/**
 * Main algorithm, creates first an initial tour, then the tour gets optimized.
 * Returns the new tour
 */ 
std::vector<autonomous_robot::TransportTask> RAI::calculateTour(){
    std::vector<autonomous_robot::TransportTask> s;
    int size = availableTasks.size();

    //Start with random Node
    int rnd = getRandomBetween(0,size-1);

    s.push_back(availableTasks[rnd]);   //Tour s starts and ends with this initial task
    s.push_back(availableTasks[rnd]);

    while ( s.size() != (size+1) ){ //+1 because first node was inserted twice
        //Get next task who is not already on tour s
        autonomous_robot::TransportTask& nextTask = getVertexNotOnTour(s);

        //Add new task in the most cheap way
        s = insertCheap(s,nextTask);
    }

    std::cout << "first tour: " << std::endl;
    printTour(s);
    std::cout << "Start optimization " << std::endl;
    std::vector<autonomous_robot::TransportTask> optimizedTour = optimizeTour(s);

    std::cout << "Optimized Tour: " << std::endl;
    printTour(s);

    return s;
}

/**
 * Choose randomly one of the tasks from availabelTask who is not on tour s
 */ 
autonomous_robot::TransportTask& RAI::getVertexNotOnTour(std::vector<autonomous_robot::TransportTask>& s){
    if ( s.size() == availableTasks.size()+1 ){
        std::cout << "No task left ... " << std::endl;
    }

    bool found = false;
    int rnd;
    do{
        found = false;
        rnd = getRandomBetween(0,availableTasks.size()-1);  //Index for task
        //Check if task with this index is already used
        for( int i = 0; i < s.size(); i++){
            if ( s[i].id == availableTasks[rnd].id){
                found = true;
            }
        }
    }while(found);
    return availableTasks[rnd];
}

std::vector<autonomous_robot::TransportTask> RAI::insertCheap(std::vector<autonomous_robot::TransportTask>& s,autonomous_robot::TransportTask nextTask){
    std::vector<autonomous_robot::TransportTask> bestSolution;

    int minCost = -1;

    //Special case 1: tour s is empty, extend it to an valid tour
    if ( s.size() == 0){
        logStream << "Tour was empty";
        // std::cout << "Tour was empty" << std::endl;
        log("insertCheap");
        bestSolution.push_back(nextTask);   //twice for start and endpoint of tour s
        bestSolution.push_back(nextTask);
        return bestSolution;
    }

    //Special case 2: tour s has only one element, extend it to an valid tour
    if( s.size() == 1){
        std::cout << "Tour was not closed " << std::endl;
        bestSolution.push_back(nextTask);
        return bestSolution;
    }

    int rounds = s.size()-1;

    //insert the nextTask on every position between two tasks in tour s.
    //
    for( int i = 0; i < rounds; i++){
        
        
        //Here, task gets inserted on every positon along s, this is done by first inserting s on pos i, 
        //then in the next round delete old s at pos i and insert it at pos i+1
        //It does not matter that nextTask gets not inserted at index 0, because it only has to be
        //between each pair of tasks, order ist'n relevant
        if ( i > 0 ){
            s.erase(s.begin()+i);
        }
        s.insert(s.begin()+i+1,nextTask);

        if( minCost == -1){
            //First solution, store it
            bestSolution = s;
            minCost = getCostForTour(s);
        }else{
            double tmpCost = getCostForTour(s);
            if ( tmpCost < minCost ){
                bestSolution = s;
                minCost = tmpCost;
            }
        }

    }
    return bestSolution;
}

std::vector<autonomous_robot::TransportTask> RAI::optimizeTour(std::vector<autonomous_robot::TransportTask> originalTour){
    int max = availableTasks.size()-1;
    int size = availableTasks.size();

    double originalTourCost = getCostForTour(originalTour);

    //std::cout << "[optimizeTour] " << " originalTourCost: " << originalTourCost << " AvailableTaskSize: " << size << " #Rounds: " << pow(size,2) << std::endl;
    logStream << "originalTourCost: " << originalTourCost << " AvailableTaskSize: " << size << " #Rounds: " << pow(size,2);
    log("optimizeTour");

    for( int counter = 0; counter < pow(size,2); counter++){
        logStream << "Round " << counter;
        log("optimizeTour");
        // std::cout << "[optimizeTour] Round " << counter << std::endl;
        //Copy original Tour, make only changes on copied tour
        std::vector<autonomous_robot::TransportTask> currentTour = originalTour;

        //Get two random indices
        int i = getRandomBetween(0,max);
        int j = getRandomBetween(0,max);

        //If i > j, change variables
        if( i > j){
            int tmp = i;
            i = j;
            j = tmp;
        }
        //delete all tasks between i and j
        
        logStream << "i: " << i << " j: " << j;
        log("optimizeTour");
        // std::cout << "[optimizeTour] i: " << i << " j: " << j << std::endl;
        logStream << "Tour before erase";
        log("optimizeTour");
        // std::cout << "[optimizeTour] Tour before erase" << std::endl;
        if( showLogs ){
            printTour(currentTour);
        }
        currentTour.erase(currentTour.begin()+i,currentTour.begin()+j+1);
        logStream << "Tour after erase";
        log("optimizeTour");
        // std::cout << "[optimizeTour] Tour after erase" << std::endl;
        if( showLogs ){
            printTour(currentTour);
        }
        //make it a valid tour again. Because the tour starts and ends with the same task, check if either of this tasks was deleted

        if( i == 0 && j == max){
            //  std::cout << "[optimizeTour] i == j, clear tour" << std::endl;
            logStream << "i == 0 && j == max";
            log("optimizeTour");
            //Complete tour was deleted, delete last Element
            currentTour.clear();
        }else{
            if ( i == 0){
                //  std::cout << "[optimizeTour] i == 0 found" << std::endl;
                logStream << "i == 0 found";
                log("optimizeTour");
                //delete last Element as well
                currentTour.erase(currentTour.begin()+currentTour.size()-1);
                //Add first task at the end of the list to make a valid tour again
                currentTour.push_back(currentTour[0]);
            }
        }
        logStream << "Tour after Checks: ";
        log("optimizeTour");
        //  std::cout << "[optimizeTour] Tour after checks:" << std::endl;
        if ( showLogs ){
            printTour(currentTour);
        }
        //Make the new tour complet
        //  std::cout << "[optimizeTour] Make the new tour complete " << std::endl;
        logStream << "Make the new tour complete ";
        log("optimizeTour");
        while( currentTour.size() != (size+1)){ //+1, because tour has first task also at the end to make tour complete
            autonomous_robot::TransportTask& task = getVertexNotOnTour(currentTour);
            //  std::cout << "[optimizeTour] Insert new task: " << task.id << std::endl;
            logStream << "Insert new Task : " << task.id;
            log("optimizeTour");
            currentTour = insertCheap(currentTour,task);
            //  std::cout << "[optimizeTour] Task inserted:" << std::endl;
            logStream << "Task inserted";
            log("optimizeTour");
            if( showLogs ){
                printTour(currentTour);
            }
        }


        //Keep the tour with lowest cost
        double currentTourCost = getCostForTour(currentTour);
        // std::cout << "New Tour with cost " << currentTourCost << std::endl;
        logStream << "New tour with cost " << currentTourCost;
        log("optimizeTour");
        if ( showLogs ){
            printTour(currentTour);
        }
        if ( currentTourCost < originalTourCost ){
            originalTour = currentTour;
            originalTourCost = currentTourCost;
            // std::cout << "New Tour has lower cost " << std::endl;
            logStream << "New Tour has lower cost ";
            log("optimizeTour");
        }else{
            // std::cout << "Old Tour has lower cost" << std::endl;
            logStream << "Old Tour has lower cost";
            log("optimizeTour");
        } 
    }
    return originalTour;
}

/**
 * Change the tour so that it is in correct order for the robot, depending on 
 * his current task in execution/ position of robot
 */ 
void RAI::changeOrder(std::vector<autonomous_robot::TransportTask>& tour){
    //delete last element
    tour.erase(tour.begin()+(tour.size()-1));
    //Change order 
    bool finished = false;
    do{
        finished = (tour.at(0).id == currentExecutionTask.id);

        if ( !finished ){
            //Copy first element at the end
            tour.push_back(tour.at(0));
            //Delete first element
            tour.erase(tour.begin());
        
        }
    }while(!finished);

    //Delete first element, it is no longer needed
    tour.erase(tour.begin());
}
    

/**
 * Calculates the cost for the given tour s, cost = distance of deadhead betweeen tasks
 */ 
double RAI::getCostForTour(std::vector<autonomous_robot::TransportTask>& s){
   double totalCost = 0;
    for( int i = 0; i < (s.size()-1);i++){
        int indexFirstTask = getIndexForTask(s[i]);
        int indexSecondTask = getIndexForTask(s[i+1]);
        totalCost += costMatrix[indexFirstTask][indexSecondTask];
    }
    return totalCost;
}

double RAI::getCostForTourExecutionTime(std::vector<autonomous_robot::TransportTask>& s){
    double totalCost = 0;
    for( int i = 0; i < (s.size()-1);i++){
        int indexFirstTask = getIndexForTask(s[i]);
        int indexSecondTask = getIndexForTask(s[i+1]);
        //Cost to drive to tour
        totalCost += costMatrix[indexFirstTask][indexSecondTask];

        //Cost for tour itself
        totalCost += costMatrix[indexFirstTask][indexFirstTask];
    }
    return totalCost;
}

/**
 * Calculates the cost for the given tour s, cost = distance of deadhead betweeen tasks, for external tours
 */ 
double RAI::calculateCostForTour(std::vector<autonomous_robot::TransportTask>& s){
    double totalCost = 0;
    if( s.size() == 0 || s.size() == 1){
        return 0;
    }
    for( int i = 0; i < (s.size()-1);i++){
        autonomous_robot::TransportTask task1 = s[i];
        autonomous_robot::TransportTask task2 = s[i+1];
        //task1.endpoint -> task2.startpoint
        totalCost += abs(task1.endPoint.x-task2.startPoint.x) + abs(task1.endPoint.y-task2.startPoint.y);
    }
    return totalCost;
}


/**
 * Get the cost, cost = execution time(distance) for the task itself + deadheadtime(distance)
 */ 
double RAI::calculateCostForTourExecutionTime(std::vector<autonomous_robot::TransportTask>& s){
    double totalCost = 0;
    
    if( s.size() == 0){
        return 0;
    }

    if( s.size() == 1){
        //The time the robot needs to do his current task. Could be exacter if the robot knows how much time is left for the current task instead of the whole execution time
        autonomous_robot::TransportTask task = s[s.size()-1];
        totalCost += abs(task.startPoint.x - task.endPoint.x) + abs(task.startPoint.y-task.endPoint.y);
        return totalCost;
    }

    for( int i = 0; i < (s.size()-1);i++){
        autonomous_robot::TransportTask task1 = s[i];
        autonomous_robot::TransportTask task2 = s[i+1];
        //Time to drive from task1 to task2
         totalCost += abs(task1.endPoint.x-task2.startPoint.x) + abs(task1.endPoint.y-task2.startPoint.y);

        //Add cost for current task itself (task1)
        totalCost +=  abs(task1.startPoint.x - task1.endPoint.x) + abs(task1.startPoint.y-task1.endPoint.y);
    }

    //Cost for last task is missing
    autonomous_robot::TransportTask lastTask = s[s.size()-1];
    totalCost += abs(lastTask.startPoint.x - lastTask.endPoint.x) + abs(lastTask.startPoint.y-lastTask.endPoint.y);

    return totalCost;
}

int RAI::getIndexForTask(autonomous_robot::TransportTask task){
    for(int i = 0; i < availableTasks.size(); i++){
        if ( task.id == availableTasks[i].id ){
            return i;
        }
    }
    return -1;
}

int RAI::getRandomBetween(int min, int max){
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::uniform_int_distribution<int> dist(min, max);
    return dist(engine);
}

void RAI::printTour(std::vector<autonomous_robot::TransportTask>& s){
    std::stringstream ss;

    for(int i = 0; i < s.size(); i++){
        ss << s[i].id << " ";
    }
    std::cout << ss.str() << std::endl;
}

void RAI::log(std::string function){
    if ( showLogs ){
        std::cout << "[RAI][" <<  function << "] " << logStream.str() << std::endl;
        logStream.str( std::string() );
	    logStream.clear();
    }
}

RAI::RAI() {
	// TODO Auto-generated constructor stub

}


RAI::~RAI() {
	// TODO Auto-generated destructor stub
}

