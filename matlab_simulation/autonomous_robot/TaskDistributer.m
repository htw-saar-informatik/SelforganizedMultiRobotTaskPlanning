classdef TaskDistributer
   %masterRobot receives tasks and sends out offer to all other
   %robots, after receiving all counteroffers and calculating the winner, the robots are informed of
   %their win or loss
   properties (Access = public)
      transportTasks = []
      offerResults = []
      errorRobots = []
      notifiedCounterOffer = 0
      taskNr  = 0
      notified = 0
      executionTimeForRobots = []
      totalCostDeadheadDistance = 0
      totalCalculationTime = 0.0
      finishedTasks = rosmessage('autonomous_robot/TransportTask')
      csvWritten = false
   end
   methods (Access = public)
      function startTaskProcessing (obj,robotCount,node,anzahlRob,anzahlAuftraege,costPerformance)
          processTasks(obj,robotCount,node,anzahlRob,anzahlAuftraege,costPerformance);
      end     
      %Callbackfunction, collects all counteroffers
      function [obj] = callbackReceiveCounterOffer(obj,~,req,~,robotCount)
          global shared;
          global distributer;
          
          %all received results are added to a list
          distributer{robotCount,1}.offerResults = [distributer{robotCount,1}.offerResults req];
          listRobots = shared{robotCount,1}.robotNames(:,1);
          listRobots = listRobots.';
          availableRobots = length(listRobots);
%           errorRobotsCount = length(distributer{robotCount,1}.errorRobots);
%           difference = availableRobots - errorRobotsCount;
           
          %notifiedCounterOffer is only true if the sum of the
          %counteroffers and the robots not reached, equals the number of
          %available robots
           if length(distributer{robotCount,1}.offerResults) == availableRobots
               distributer{robotCount,1}.notifiedCounterOffer = true;
           end
      end
      %Callbackfunction, collects all new added tasks
      function [response] = callbackAddNewTransportTask(~,~,req,response,robotCount)
        global shared;
        global distributer;
        %received task gets a new Id and the task is then added to the list
        %of available tasks
        req.Task.Id = shared{robotCount,1}.getNextId();
        distributer{robotCount,1}.transportTasks = [distributer{robotCount,1}.transportTasks req.Task];

        distributer{robotCount,1}.notified = true;
        response.Success = 0;
      end
      
      %Callbackfunction, collects all finished tasks
      function [obj] = callbackReceiveFinishedTask(obj,~,req,~,robotCount)
          global distributer;
          
          %task added to list of finished tasks
          distributer{robotCount,1}.finishedTasks = [distributer{robotCount,1}.finishedTasks req.Task];
      end
   end  
   methods (Access = private)
      %starts the process
      function processTasks (~, robotCount,node,anzahlRob,anzahlAuftraege,costPerformance) 
          global distributer;
          distributer{robotCount,1}.runTask(robotCount,node,anzahlRob,anzahlAuftraege,costPerformance);
      end
      
      %waits till a new tasks arrives and sends the offer to all known robots
      function runTask(~,robotCount,node,anzahlRob,anzahlAuftraege,costPerformance)
          global distributer;
          global shared;
          
          longestExecutionTime = 0;
          %start timer
          tic;
          
          %checks if list of tasks is empty from the start before beginning
          %the endless loop
          if ~isempty(distributer{robotCount,1}.transportTasks)
              distributer{robotCount,1}.notified = true;
          end
          
          while true  
              %waiting to get notified by a new task
              while ~distributer{robotCount,1}.notified
                  drawnow;
                  %after all tasks were distributed the result is written in a csv file
                  if distributer{robotCount,1}.taskNr == shared{robotCount,1}.countTasksForClientAutomation && distributer{robotCount,1}.taskNr ~= 0
                      if length(distributer{robotCount,1}.finishedTasks) == shared{robotCount,1}.countTasksForClientAutomation+1 && ~distributer{robotCount,1}.csvWritten
                          distributer{robotCount,1}.output(distributer{robotCount,1}.totalCostDeadheadDistance,distributer{robotCount,1}.totalCalculationTime,longestExecutionTime,anzahlRob,anzahlAuftraege,costPerformance);
                          distributer{robotCount,1}.csvWritten = true;
                      end
                  end
              end
              
              leave = false;
              while ~leave
                  %if no task available continue waiting again
                  if isempty(distributer{robotCount,1}.transportTasks)
                      leave = true;
                      distributer{robotCount,1}.notified = false;
                      drawnow;
                  else
                      %if task available send to all robots and remove from
                      %list
                      task = distributer{robotCount,1}.transportTasks(1,1);
                      distributer{robotCount,1}.transportTasks(:,1) = [];
                      longestExecutionTime = distributer{robotCount,1}.sendOfferToAllRobots(task,robotCount,node,anzahlRob,anzahlAuftraege,costPerformance);
                      pause(5);
                      drawnow;
                  end
              end
              drawnow;
          end
      end
      
      %offer is sent to all other robots and after receiving all
      %counteroffers, the winner is calculated and the robots are
      %informed of their win or loss, if all tasks have been proccessed,
      %the results are written to a csv-file
      function result = sendOfferToAllRobots(~, task,robotCount,node,anzahlRob,anzahlAuftraege,costPerformance)
          global shared;
          global distributer;
          distributer{robotCount,1}.offerResults = [];
          distributer{robotCount,1}.errorRobots = [];
          distributer{robotCount,1}.notifiedCounterOffer = false;
          
          %Client is called for each robot's individual ROS-Server
          listRobots = shared{robotCount,1}.robotNames(:,1);
          listRobots = listRobots.';
          for name=listRobots
              try
                  serverName = char(strcat('/ReceiveOffer', int2str(name)));
                  Client = ros.ServiceClient(node,serverName);
                  request = rosmessage(Client);
                  request.Task = task;
                  request.From = int2str(shared{robotCount,1}.nodeID);
                  request.To = int2str(name);
                  response = call(Client,request,'Timeout',5);
                  drawnow;
              catch ME
                  msg = ['could not reach other robot to let them receive offer']
                  distributer{robotCount,1}.errorRobots = [distributer{robotCount,1}.errorRobots name];
              end
          end
          
          %if none of the robots have been reached, the task is again added
          %to the list of available tasks
          if length(distributer{robotCount,1}.errorRobots) == length(listRobots)
              distributer{robotCount,1}.transportTasks = [distributer{robotCount,1}.transportTasks task];
              return;
          end

        %waiting for all counteroffers
        while ~distributer{robotCount,1}.notifiedCounterOffer 
            drawnow;
        end
        
        minCost = -1;
        winnerRequest = [];
        %find offer with minimal distanz/executionTime
        if shared{robotCount,1}.distributionMode == 0
            for req=distributer{robotCount,1}.offerResults
                if minCost == -1
                   minCost = (req.CostWithTask - req.CostWithoutTask);
                    winnerRequest = req; 
                else
                    tmpCost = (req.CostWithTask - req.CostWithoutTask);
                    if tmpCost < minCost
                       minCost = tmpCost;
                       winnerRequest = req;
                    end
                end
            end
        else
            for req = distributer{robotCount,1}.offerResults
                if minCost == -1
                   minCost = req.CostWithTaskExecutionTime;
                   winnerRequest = req; 
                else
                   if req.CostWithTaskExecutionTime < minCost
                       minCost = req.CostWithTaskExecutionTime;
                       winnerRequest = req;
                   end
                end
            end
        end

        %the time and distance so far are being calculated
        distributer{robotCount,1}.totalCostDeadheadDistance = distributer{robotCount,1}.totalCostDeadheadDistance +(winnerRequest.CostWithTask - winnerRequest.CostWithoutTask);
        writematrix(distributer{robotCount,1}.totalCostDeadheadDistance,'cot.csv','WriteMode','append');
        distributer{robotCount,1}.totalCalculationTime = toc;
        distributer{robotCount,1}.executionTimeForRobots = winnerRequest.ApproximatedExecutionTime;

        %informing the winner
        winnerinformed = 0;
        while winnerinformed ~=1
            try
                winnerinformed = 1;
                serverName = char(strcat('/WinOffer', winnerRequest.From));
                Client = ros.ServiceClient(node,serverName);
                response = call(Client,'Timeout',5);
                drawnow;
            catch ME
                msg = ['master could not inform winner'];
                winnerinformed = 0;
            end
        end

        %inform all others of loss
        for name=listRobots
            if ~strcmp(int2str(name),winnerRequest.From)
                informed = 0;
                while informed ~= 1
                    try
                        informed = 1;
                        serverName = char(strcat('/LooseOffer',int2str(name)));
                        Client = ros.ServiceClient(node,serverName);
                        response = call(Client,'Timeout',5);
                        drawnow;
                    catch ME
                        msg = ['master could not inform robot of loss']
                        informed = 0;
                    end
                end   
            end
        end

        distributer{robotCount,1}.taskNr = distributer{robotCount,1}.taskNr +1;

        %finds the longest executionTime
        longestExecutionTime = 0;
        executionTimeList = distributer{robotCount,1}.executionTimeForRobots;
        length(executionTimeList)
        for i = length(executionTimeList)
            if distributer{robotCount,1}.executionTimeForRobots(1,i) > longestExecutionTime
                longestExecutionTime = distributer{robotCount,1}.executionTimeForRobots(1,i);
            end
        end
        
        result = longestExecutionTime;
        
      end
      
      %results are saved in a csv-file
      function output(~,totalCostDeadheadDistance, totalCalculationTime, totalExecutionTime,anzahlRob,anzahlAuftraege,costPerformance)
          filename = ['dynamic',num2str(anzahlRob),'roboter',num2str(anzahlAuftraege),'task',num2str(costPerformance),'goal','.csv'];

          data = [totalCostDeadheadDistance totalExecutionTime totalCalculationTime  toc];
          writematrix(data,filename,'WriteMode','append');
      end
      
   end
end