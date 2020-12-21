classdef RAI
    %RAI class calculates a costmatrix and with its help creates a tour
    %which includes all vertexes(robots and tasks) by inserting a vertex, not currently included, in the cheapest way;
    %afterwards some vertexes are removed and inserted again in the
    %cheapest way, potentially optimizing the initial tour, this is done a
    %set amount of times
    methods
        %creates matrix by calculating the cost between each vertex
        function result = createMatrix(obj, anzahlRob, anzahlAuftraege, robotList, taskList)
            for i = 1:anzahlRob + anzahlAuftraege
                for j = 1:anzahlAuftraege + anzahlRob
                    matrix(i,j) = obj.calculateMatrix(i,j, anzahlRob, robotList, taskList);    
                end
            end
            result = matrix; 
        end
        %calculates cost between two indices depending on them being robots or tasks 
        function result = calculateMatrix(~, firstIndex, secondIndex, anzahlRob, robotList, taskList)
            %if both indices are the same robot/task cost equals -1
            if eq(firstIndex, secondIndex)
                result = -1;
                return;
            end
            %if both indices are robots the costs are 0
            if (firstIndex <= anzahlRob) && (secondIndex <= anzahlRob)
                result = 0;
                return;
            end
            %the first index is a robot and the second is a task
            if (firstIndex <= anzahlRob && secondIndex > anzahlRob)
                resultx = abs(robotList(firstIndex, 2) - taskList(secondIndex - anzahlRob, 2));
                resulty = abs(robotList(firstIndex, 3) - taskList(secondIndex - anzahlRob, 3));
                result = resultx + resulty;
                return;
            end
            %both indices are tasks
            if (firstIndex > anzahlRob && secondIndex > anzahlRob)
                resultx = abs(taskList(firstIndex - anzahlRob, 2) - taskList(secondIndex - anzahlRob, 2));
                resulty = abs(taskList(firstIndex - anzahlRob, 3) - taskList(secondIndex - anzahlRob, 3));
                result = resultx + resulty; 
                return;
            end
            %if first index is a task and second is a robot the costs are 0
            if (firstIndex > anzahlRob && secondIndex <= anzahlRob)
                result = 0;
                return;
            end
        end
        
        %starting the algorithm that creates and optimizes a tour
        function result = startAlgo(obj, anzahlRob, anzahlAuftraege, matrix, costPerformance, taskList, roundsOptimization)
            %calculating lowerBound first
            lowerBoundDistance = obj.calculateLowerBoundDistance(anzahlAuftraege, anzahlRob, matrix);
            lowerBoundExecutionTime = obj.calculateLowerBoundExecutionTime(anzahlAuftraege,anzahlRob, matrix, taskList);
            
            optimizedTour = 0;
            lengthArray = anzahlRob + anzahlAuftraege;
            %choosing 2 random vertexes
            rnd(1,1)= obj.getRandomNumberInRange(1, lengthArray );
            rnd(1,2)= obj.getRandomNumberInRange(1, lengthArray );
            
            %if both are the same vertex, randomly choose a new vertex
            while rnd(1,1) == rnd(1,2)
                rnd(1,2)= obj.getRandomNumberInRange(1, lengthArray );
            end

            %gets a random vertex and includes it in the cheapest way into
            %the tour until it is complete
            while ~eq(length(rnd), lengthArray)
                nextVertex = obj.getVertexNotOnTour(rnd, lengthArray);
                currentTourInsert = obj.insertCheap(rnd, nextVertex, costPerformance,matrix, anzahlRob, taskList);
                while obj.checkIfTourIsValid(currentTourInsert) == false
                    nextVertex = obj.getVertexNotOnTour(rnd, lengthArray);      
                    currentTourInsert = obj.insertCheap(rnd, nextVertex, costPerformance,matrix, anzahlRob, taskList);
                end
                rnd = currentTourInsert;
            end
         
            %optimizes tour
            optimizedTour = obj.optimizeTour(rnd, anzahlRob, anzahlAuftraege, roundsOptimization, costPerformance, matrix, taskList);
            tour = optimizedTour;
            
            %to fully complete tour add first vertex at the end
            optimizedTour = [optimizedTour optimizedTour(1,1)];
            
            
            %calculate the distance and executionTime
            tourDistance =  obj.getTourDistance(optimizedTour ,matrix);
            tourExecutionTime =  obj.getTourExecutionTime(optimizedTour ,matrix, anzahlRob, taskList);
            
            result = [tourDistance, tourExecutionTime, tour];
        end
        
        function lowerBoundDistance = calculateLowerBoundDistance (~, anzahlAuftraege, anzahlRob, matrix)
            minValue = 0;
            value = 0;
            lowerBound = 0;

            %chooses the lowest value in matrix between 2 vertexes and adds
            %them together to create lowerBoundDistance
            for i = 1:anzahlAuftraege
                minValue = -1;
                for j = 1:anzahlRob + anzahlAuftraege
                    value = matrix(j, i+anzahlRob);
                    if value ~= -1
                        if minValue == -1
                            minValue = value;
                        elseif value < minValue
                                minValue = value;
                        end
                    end
                end
                lowerBound = lowerBound + minValue;
            end
            lowerBoundDistance = lowerBound;
        end
        function lowerBoundExecutionTime = calculateLowerBoundExecutionTime(obj, anzahlAuftraege, anzahlRob, matrix, taskList)
                minValue = 0;
                minValues = [];
                globalMinValue = -1;
                value = 0;
                
                %similar to the lowerBoundDistance the lowest value between
                %2 vertexes is chosen, the task duration is additionally
                %added
                for i=1:anzahlAuftraege
                    minValue = -1;
                    for j=1:anzahlRob
                        value = matrix(j,i);
                        value = value + obj.getTaskDuration(taskList, i);
                        if value ~= -1
                            if minValue == -1
                                minValue = value;
                            else
                                if matrix(j,i) ~= -1
                                    if value < minValue
                                        minValue = value;
                                    end
                                end
                            end
                        end
                        minValues = [minValues minValue];

                        if globalMinValue == -1
                            globalMinValue = minValue;
                        else
                            if globalMinValue > minValue
                            globalMinValue = minValue;
                            end
                        end
                    end
                end

                minTaskCount = ceil(anzahlAuftraege / anzahlRob);
                lowestSum = 0;
                for k=1:minTaskCount
                    lowestSum = lowestSum + minValues(1);
                    minValues(1) = [];
                end

                if globalMinValue > lowestSum
                    lowerBoundExecutionTime = globalMinValue;
                else
                    lowerBoundExecutionTime = lowestSum;
                end

        end 
        function result = getTaskDuration(~, taskList, i)
            result = abs(taskList(i, 2) - taskList(i,4)) + abs(taskList(i,3) - taskList(i,5));
        end

        function rnd = getRandomNumberInRange(~, min, max)
            %rnd = min + (max+min).*randi(1,1)
            rnd = randi([min max], 1);
        end   
        function result = getVertexNotOnTour(obj, rnd, lengthArray)
                found = false;
                lengthRnd = length(rnd);
                
                %chooses a random vertex that is not already found in the
                %current tour
                while true
                     found = false;
                     random = obj.getRandomNumberInRange(1, lengthArray );
                     for i=1:lengthRnd
                         if rnd(1,i) == random
                             found = true;
                         end
                     end
                    if ~found
                        break
                    end
                end
                result = random;
        end   
        function result = insertCheap (obj, rnd, nextVertex, costPerformance, matrix, anzahlRob, taskList)
            bestSolution = [];
            minCosts = -1;
            tour = rnd;
            %if the tour is empty, the current vertex is added to the
            %bestSolution
            if isempty(rnd)
                bestSolution = [bestSolution nextVertex];
                result = bestSolution;
                return;
            end
            
            %if the tour includes only one element, the current vertex is
            %added
            if length(tour) == 1
                tour = [tour nextVertex];
                result = tour;
                return;
            end

            rounds = length(tour) - 1;
            counter = 0;
            
            %for a set amount of rounds the nextVertex is included
            %inbetween all other vertexes and the solution that costs less
            %is chosen as the bestSolution
            for i=1:rounds
                if counter > 0
                    tour(i) = [];
                end
                tour = [tour(1,1:i) nextVertex tour(1,i+1:end)];
                if minCosts == -1
                    bestSolution = tour;
                    minCosts = obj.getCostsForTour(tour, costPerformance,matrix, anzahlRob, taskList);
                    %bestsolutionmincostminus = bestSolution
                end
                if minCosts ~= -1
                  tmpDistance = obj.getCostsForTour(tour, costPerformance,matrix, anzahlRob, taskList);
                  if tmpDistance < minCosts
                      bestSolution = tour;
                      minCosts = tmpDistance;
                      %bestsolutionelse = bestSolution
                  end
                end
                counter = counter +1;    
            end
            result = bestSolution;
        end
        
        %depending on the costPerformance the tour will be optimized for
        %either distance or executionTime
        function result = getCostsForTour (obj, tour, costPerformance,matrix, anzahlRob, taskList)
            if costPerformance == 0 
                result = obj.getTourDistance(tour,matrix);
                return;
            elseif costPerformance == 1
                result = obj.getTourExecutionTime(tour,matrix, anzahlRob, taskList);
                return;
            else
                result = obj.getTourDistance(tour,matrix);
            end
        end 
        function result = getTourDistance(~, tour,matrix)
            totalDistance = 0;
            %values in the matrix are added together to calculate the
            %distance
            for i=1:length(tour)-1
                totalDistance = totalDistance + matrix(tour(1,i),tour(1,i+1));
            end
            result = totalDistance;
        end
        function result = getTourExecutionTime(obj, tour, matrix, anzahlRob, taskList)
            maxExecutionTime = 0;
            currentExecutionTime = 0;
            executionTimeBeginning = 0;
            hasSetExecutionTimeBeginning = false;
            firstValue = tour(1,1);
            lastRobot = 0;

            currentValue = 0;
            nextValue = 0;

            %the calculation begins when the first robot is found
            for i=1:length(tour)-1
                currentValue = tour(1,i);
                nextValue = tour(1,i+1);
                if obj.checkIsRobot(currentValue, anzahlRob)
                    if ~obj.checkIsRobot(firstValue, anzahlRob)
                        if ~hasSetExecutionTimeBeginning
                            executionTimeBeginning = currentExecutionTime;
                            hasSetExecutionTimeBeginning = true;
                        end
                    end

                    if currentExecutionTime > maxExecutionTime
                        maxExecutionTime = currentExecutionTime;
                    end
                    currentExecutionTime = 0;
                    lastRobot = currentValue;
                    currentExecutionTime = currentExecutionTime + matrix(currentValue,nextValue);

                else
                    erg = abs(taskList(currentValue - anzahlRob, 2)-taskList(currentValue - anzahlRob, 4)) + abs(taskList(currentValue - anzahlRob, 3)- taskList(currentValue - anzahlRob, 5));
                    currentExecutionTime = currentExecutionTime + erg;

                    currentExecutionTime = currentExecutionTime + matrix(currentValue,nextValue);

                end
            end
            if length(tour) - 1 > 0
                secondToLastValue = tour(1,length(tour) - 1);
            else
                if length(tour) == 2
                secondToLastValue = tour(1,1);
                end
                if length(tour) == 1
                  secondToLastValue = tour(1,1);
                end
            end

            if obj.checkIsRobot(secondToLastValue, anzahlRob)
                lastRobot = secondToLastValue;
                if ~obj.checkIsRobot(firstValue, anzahlRob)
                    if ~hasSetExecutionTimeBeginning
                        executionTimeBeginning = currentExecutionTime;
                    end
                end

                if currentExecutionTime > maxExecutionTime
                    maxExecutionTime = currentExecutionTime;
                end
                currentExecutionTime = 0;
            else 
                 erg = abs(taskList(secondToLastValue - anzahlRob, 2)-taskList(secondToLastValue - anzahlRob, 4)) + abs(taskList(secondToLastValue - anzahlRob, 3)- taskList(secondToLastValue - anzahlRob, 5));
                 currentExecutionTime = currentExecutionTime + erg;
                if currentExecutionTime > maxExecutionTime
                    maxExecutionTime = currentExecutionTime;
                end
            end

            if ~obj.checkIsRobot(firstValue, anzahlRob)
                if executionTimeBeginning > 0
                    currentExecutionTime = currentExecutionTime + executionTimeBeginning;
                    currentExecutionTime = currentExecutionTime + matrix(tour(1,length(tour)-2), tour(1, length(tour)-1));
                    if currentExecutionTime > maxExecutionTime
                        maxExecutionTime = currentExecutionTime;
                    end
                end
            end
            result = maxExecutionTime;
        end
        function result = checkIsRobot(~, index, anzahlRob)
            result = false;
            if index <= anzahlRob
                result = true;
                return;
            end  
        end
        %tour is valid when it is not empty and a vertex is found only once
        function result = checkIfTourIsValid(~, tour)
            tmp = [];
            counter = 0;

            if length(tour) == 0
                 result = false;
                 return;
            end

            for i=tour
                if ismember(i,tmp)
                    tmpIndex = find(tmp == i);
                    if tmpIndex == 1 && counter == length(tour)-1

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
        
        function result = optimizeTour(obj, originalTour, anzahlRob, anzahlAuftraege,roundsOptimization, costPerformance,matrix, taskList)
                max = anzahlRob + anzahlAuftraege - 1;
                Length = anzahlRob + anzahlAuftraege;
                originalTourCosts = obj.getCostsForTour(originalTour,costPerformance,matrix, anzahlRob, taskList);

                %for a set amount of times the tour is optimized
                for counter=1:roundsOptimization
                    %arraylist
                    currentTour = originalTour;

                    %2 ranom vertexes are chosen and each vertex inbetween
                    %removed
                    i = obj.getRandomNumberInRange(1, max);
                    j = obj.getRandomNumberInRange(1, max);

                    if (i > j) 
                        tmp = i;
                        i = j;
                        j = tmp;
                    end
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

                    currentTourInsert = [];

                    %each vertex is added again in the cheapest way
                   while (length(currentTour) ~= (anzahlRob + anzahlAuftraege))

                        nextVertex = obj.getVertexNotOnTour(currentTour, anzahlRob + anzahlAuftraege);
                        currentTourInsert = obj.insertCheap(currentTour, nextVertex, costPerformance, matrix, anzahlRob, taskList);
                        
                        while obj.checkIfTourIsValid(currentTourInsert) == false
                            obj.checkIfTourIsValid(currentTourInsert)
                            nextVertex = obj.getVertexNotOnTour(currentTour, anzahlRob + anzahlAuftraege);
                            currentTourInsert = obj.insertCheap(currentTour, nextVertex, costPerformance, matrix, anzahlRob, taskList);
                        end
                        currentTour = currentTourInsert;
                   end

                   %the costs for the new tour are compared to the costs
                   %for the initial tour, the lower value is chosen as the
                   %initial tour for the next round
                    currentTourCosts = obj.getCostsForTour(currentTour,costPerformance,matrix, anzahlRob, taskList);
                    if (currentTourCosts < originalTourCosts)
                        originalTour = currentTour;
                        originalTourCosts = currentTourCosts;
                    end
                end

                result = originalTour;
        end
    end
end

