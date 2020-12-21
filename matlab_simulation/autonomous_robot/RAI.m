classdef RAI
    %creates costmatrix and with its help calculates the cost for tours
    properties (Access = public)
        costMatrix
        newTask = rosmessage('autonomous_robot/TransportTask')
        availableTasks
        currentExecutionTask = rosmessage('autonomous_robot/TransportTask')
        costDistance = 0
        costExecutionTime = 0
    end
    methods (Access = public)
        function calculateOrderWithNewTask(~,task, currentExecutionTask, robotCount)
            global rai;
            global offerManager;
            global shared;
            rai{robotCount,1}.newTask = task;
            rai{robotCount,1}.currentExecutionTask = currentExecutionTask;

            %create matrix, calculate tour and calculate the costs for new
            %tour
            rai{robotCount,1}.createCostMatrix(robotCount);
            tour = rai{robotCount,1}.calculateTour(robotCount);

            offerManager{robotCount,1}.costDistance = rai{robotCount,1}.getCostForTour(tour,robotCount); 
            offerManager{robotCount,1}.costExecutionTime = rai{robotCount,1}.getCostForTourExecutionTime(tour, robotCount);

            %change order for tour
            tour = rai{robotCount,1}.changeOrder(tour, robotCount);
            shared{robotCount,1}.calculatedOrderForNewTask = tour;
        end
        
        function result = calculateCostForTour(~,s)
            totalCost = 0;
            %no cost if tour empty or only one element
            if isempty(s) || length(s) == 1
                result = 0;
                return;
            end
            
            %calculate cost by adding the differences of starting and
            %endpoints between each set of two tasks
            for i=1:length(s)-1
                task1 = s(1,i);
                task2 = s(1,i+1);
                
                totalCost = totalCost + abs(task1.EndPoint.X-task2.StartPoint.X) + abs(task1.EndPoint.Y-task2.StartPoint.Y);
            end
            result = totalCost;
        end

        function result = calculateCostForTourExecutionTime(~,s)
            totalCost = 0;
            
            %no cost if tour empty
            if isempty(s)
                result = 0;
                return;
            end
            
            %if only one element, cost equal to difference between
            %startpoints and endpoints
            if length(s) == 1
                task = s(1,length(s));
                totalCost = totalCost + abs(task.StartPoint.X - task.EndPoint.X) + abs(task.StartPoint.Y-task.EndPoint.Y);
                result = totalCost;
                return;
            end
            
            %calculate cost by adding the differences of startpoints and
            %endpoints
            for i=1:length(s)-1
                task1 = s(1,i);
                task2 = s(1,i+1);
                totalCost = totalCost +  abs(task1.EndPoint.X-task2.StartPoint.X) + abs(task1.EndPoint.Y-task2.StartPoint.Y);
                totalCost = totalCost +  abs(task1.EndPoint.X-task1.EndPoint.X) + abs(task1.EndPoint.Y-task1.EndPoint.Y);
            end
            
            lastTask = s(1,length(s)-1);
            totalCost = totalCost + abs(lastTask.StartPoint.X - lastTask.EndPoint.X) + abs(lastTask.StartPoint.Y-lastTask.EndPoint.Y);
            result = totalCost;
        end
    end
    
    methods (Access = private)
        %creates matrix from accepted tasks, the new task and the current
        %task
        function createCostMatrix(~,robotCount)
            global rai;
            global shared;

            rai{robotCount,1}.availableTasks = shared{robotCount,1}.acceptedTasks;
            rai{robotCount,1}.availableTasks = [rai{robotCount,1}.availableTasks rai{robotCount,1}.newTask];
            if rai{robotCount,1}.currentExecutionTask.Id ~= -1
                rai{robotCount,1}.availableTasks = [rai{robotCount,1}.availableTasks rai{robotCount,1}.currentExecutionTask];
            end

            if rai{robotCount,1}.availableTasks(1,1).Id == rai{robotCount,1}.availableTasks(1,end).Id && length(rai{robotCount,1}.availableTasks) > 1
                rai{robotCount,1}.availableTasks(:,end) = [];
            end

            size = length(rai{robotCount,1}.availableTasks);
            for i=1:size
                for j=1:size
                    rai{robotCount,1}.costMatrix(i,j) = rai{robotCount,1}.calculateCostForTasks(rai{robotCount,1}.availableTasks(1,i),rai{robotCount,1}.availableTasks(1,j), robotCount);
                end
            end
        end
        
        function result = calculateCostForTasks(~,task1, task2, robotCount)
            global rai;
            %if task1 equals task2, the cost is the difference between
            %startpoint and endpoint
            if task1.Id == task2.Id
                result = abs(task1.StartPoint.X - task1.EndPoint.X) + abs(task1.StartPoint.Y-task1.EndPoint.Y);
            else
                %otherwise the cost is the difference between endpoint of task1
                %and startpoint of task2 or simply 0 if task2 is currently
                %being executed
                if task2.Id == rai{robotCount,1}.currentExecutionTask.Id
                    task2IsStartTask = true;
                else
                    task2IsStartTask = false;
                end
                if task2IsStartTask
                    result = 0;
                else
                    result = abs(task1.EndPoint.X-task2.StartPoint.X) + abs(task1.EndPoint.Y-task2.StartPoint.Y);
                end
            end
        end
        
        %creates tour by adding new task in cheapest way and then
        %optimizing it once
        function result = calculateTour (~, robotCount)
            global rai;
            s = [];
            size = length(rai{robotCount,1}.availableTasks);
            rnd = rai{robotCount,1}.getRandomBetween(1,size);
            
            s = [s rai{robotCount,1}.availableTasks(1,rnd)];
            
            %creates new tour by adding a new Vertex as cheaply as
            %possible
            while length(s) ~= size
                nextTask = rai{robotCount,1}.getVertexNotOnTour(s, robotCount);
                s = rai{robotCount,1}.insertCheap(s, nextTask, robotCount);
            end

            rai{robotCount,1}.optimizeTour(s, robotCount);
            result = s;
        end
        
        %generates random value in given range
        function result = getRandomBetween (~,min, max)
            result = randi([min max], 1); 
        end
        
        %loops until a new vertex, not on tour, is found
        function result = getVertexNotOnTour (~,s, robotCount)
            global rai;
            found = true;

            while found
                found = false;
                rnd = rai{robotCount,1}.getRandomBetween(1,length(rai{robotCount,1}.availableTasks));
 
                if isempty(s)
                    result = rai{robotCount,1}.availableTasks(1,rnd);
                    return;
                end

                for i=1:length(s)
                    if s(1,i).Id == rai{robotCount,1}.availableTasks(1,rnd).Id
                        found = true;
                    end
                end
                
            end
            
            result = rai{robotCount,1}.availableTasks(1,rnd);
        end
        
        %tries out adding task in cheapest way and returns bestSolution
        function result = insertCheap (~,s, nextTask, robotCount)
            global rai;
            bestSolution = [];
            minCost = -1;
            
            %if tour is empty/has only one element, bestSolution is just adding the new task 
            if isempty(s) || length(s) == 1
                bestSolution = [s nextTask];
                result = bestSolution;
                return;
            end

            %each round tests if adding the task in another position
            %costs less
            rounds = length(s) - 1;
            counter = 0;

            for i=1:rounds
                if counter > 0
                    s(i) = [];
                end
                s = [s(1,1:i) nextTask s(1,i+1:end)];
                
                if minCost == -1
                    bestSolution = s;
                    minCost = rai{robotCount,1}.getCostForTour(s, robotCount); 
                end
                if minCost ~= -1
                  tmpDistance = rai{robotCount,1}.getCostForTour(s, robotCount);
                  if tmpDistance < minCost
                      bestSolution = s;
                      minCost = tmpDistance;   
                  end
                end
                counter = counter +1;    
            end
            result = bestSolution;
        end
        
        %returns cost for tour with the help of the costMatrix
        function result = getCostForTour (~,s, robotCount)
            global rai;
            totalCost = 0;
            
            if length(s) <= 1
                indexFirstTask = rai{robotCount,1}.getIndexForTask(s(1,1),robotCount);
                totalCost = totalCost + rai{robotCount,1}.costMatrix(indexFirstTask,indexFirstTask);  
            else
                for i=1:length(s)-1
                    indexFirstTask = rai{robotCount,1}.getIndexForTask(s(1,i),robotCount);
                    indexSecondTask = rai{robotCount,1}.getIndexForTask(s(1,i+1),robotCount);
                    totalCost = totalCost + rai{robotCount,1}.costMatrix(indexFirstTask,indexSecondTask);
                end
            end
            result = totalCost;
        end
        
        %returns cost for tour with the help of the costMatrix, additionaly
        %adds the duration for each task to the sum
        function result = getCostForTourExecutionTime (~,s, robotCount)
            global rai;
            totalCost = 0;
            
            for i=1:length(s)-1
                indexFirstTask = rai{robotCount,1}.getIndexForTask(s(1,i),robotCount);
                indexSecondTask = rai{robotCount,1}.getIndexForTask(s(1,i+1),robotCount);
                totalCost = totalCost + rai{robotCount,1}.costMatrix(indexFirstTask,indexSecondTask);
                totalCost = totalCost + rai{robotCount,1}.costMatrix(indexFirstTask,indexFirstTask);
            end
            result = totalCost;
        end
        
        %returns the index of the given task
        function result = getIndexForTask (~,task, robotCount)
            global rai;
            for i=1:length(rai{robotCount,1}.availableTasks)
                if task.Id == rai{robotCount,1}.availableTasks(1,i).Id
                    result = i;
                    return;
                end
            end
            result = -1;
        end
        
        %removes parts of original tour by choosing 2 random numbers, then
        %inserting all missing vertexes in the cheapest way, returning tour
        %that costs less
        function result = optimizeTour (~,originalTour, robotCount)
            global rai;
            max = length(rai{robotCount,1}.availableTasks);
            size = length(rai{robotCount,1}.availableTasks);
            
            %first getting cost for original tour
            originalTourCosts = rai{robotCount,1}.getCostForTour(originalTour,robotCount);
            
            %trying every combination for inserting the new task
            for counter=1:power(size,2)
                currentTour = originalTour;
                %getting 2 random numbers, j has to be bigger in value
                i = rai{robotCount,1}.getRandomBetween(1, max);
                j = rai{robotCount,1}.getRandomBetween(1, max);

                if (i > j) 
                    tmp = i;
                    i = j;
                    j = tmp;
                end
                
                %removes values from currentTour(i) until currentTour(j)
                 x = j-i+1;
                 for f=1:x
                     currentTour(i) = [];  
                 end

                 if i == 1 && j == max
                    currentTour = [];
                else
                    if i == 1
                        currentTour(length(currentTour)) = [];
                    end
                 end
                 
               %loop until the tour is complete
               while (length(currentTour) ~= (size))
                    nextVertex = rai{robotCount,1}.getVertexNotOnTour(currentTour, robotCount); 
                    currentTourInsert = rai{robotCount,1}.insertCheap(currentTour, nextVertex, robotCount);
                    
                    %only adds new vertex if tour is valid
                    if rai{robotCount,1}.checkIfTourIsValid(currentTourInsert) == true
                        currentTour = currentTourInsert;
                    end 
               end

               %if costs for current tour lesser than cost for orignial
               %tour, current tour replaces original tour for next round
                currentTourCosts = rai{robotCount,1}.getCostForTour(currentTour,robotCount);
                if (currentTourCosts < originalTourCosts)
                    originalTour = currentTour;
                    originalTourCosts = currentTourCosts;
                end
            end
            result = originalTour;
        end
        
        %tour is valid when it is not empty and a vertex is found only once
        function result = checkIfTourIsValid(~,tour)
            tmp = [];
            counter = 0;

            if isempty(tour)
                 result = false;
                 return;
            end
            
            for i=tour
                if ismember(i,tmp)
                    tmpIndex = find(tmp == i);
                    if tmpIndex == 0 && counter == length(tour)-1
                    else
                        result = false;
                        return;
                    end
                else 
                    tmp = [tmp i];
                end
                counter = counter + 1;
            end
            result = true;
        end
        
        %the order is changed when the task in first position is finished
        %executing
        function result = changeOrder (~,tour, robotCount)
            global rai;
            finished = false;
            
            %after first task done, remove it from start and add to end
            while ~finished
                finished = tour(1,1).Id == rai{robotCount,1}.currentExecutionTask.Id;
                if rai{robotCount,1}.currentExecutionTask.Id == -1
                    finished = true;
                end
                if ~finished
                    tour = [tour tour(1,1)];
                    tour(:,1) = [];
                end
            end

            result = tour;
        end  
    end
end

