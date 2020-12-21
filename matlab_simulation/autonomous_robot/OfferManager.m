classdef OfferManager
    %processes the task assigned to a robot, simultaniously waits for new
    %offers and after calculating cost with and without the new task, sends
    %result back, waits for win or loss of said task
    properties (Access = public)
      %driveSimulator = DriveSimulator
      offerList = []
      startOfferProcess = 0
      isWaitingForOfferResult = 0
      isNotified = 0
      drivedDistance = 0
      executedTime = 0
      executedTasks = rosmessage('autonomous_robot/TransportTask')
      costDistance = 0
      costExecutionTime = 0
    end
   
    methods (Access = public)
      %starts process
      function startTaskProcessing(~,node, robotCount)
        global offerManager;
        offerManager{robotCount,1}.processTasks(node,robotCount);
        end
      %Callbackfunction, processes offers sent by masterNode
      function [response] = callbackReceiveOffer(~,~,req,response, robotCount)
       global offerManager;
       %new task is added to offerList
       offerManager{robotCount,1}.isWaitingForOfferResult = true;
       offerManager{robotCount,1}.startOfferProcess = true;
       offerManager{robotCount,1}.offerList = [offerManager{robotCount,1}.offerList req];
       offerManager{robotCount,1}.isNotified = true;
       
       response.Success = 0; 
        end
      %Callbackfunction, called when robots wins task
      function [obj] = callbackWinOffer(obj,~,~,~, shared, robotCount)
       global offerManager;
       global shared;
       %the order calculated before with the new task is set as
       %acceptedTasks
       shared{robotCount,1}.acceptedTasks = shared{robotCount,1}.calculatedOrderForNewTask;
       shared{robotCount,1}.calculatedOrderForNewTask = [];
       offerManager{robotCount,1}.isWaitingForOfferResult = false;
       offerManager{robotCount,1}.isNotified = true;
      end
      %Callbackfunction, called when robots looses task
      function [obj] = callbackLooseOffer(obj,~,~,~, shared, robotCount)
       global offerManager;
       global shared;
       %calculatedOrderForNewTask is deleted, therefore the new task is not
       %added
       offerManager{robotCount,1}.isWaitingForOfferResult = false;
       offerManager{robotCount,1}.isNotified = true;
       shared{robotCount,1}.calculatedOrderForNewTask = [];
      end
    end
    methods (Access = private)
      function processTasks(~,node, robotCount)
          global offerManager;
          global shared;
          global rai;
          rai{robotCount,1} = RAI;

          while true 
              %waiting until robot gets notified
              while ~offerManager{robotCount,1}.isNotified
                  drawnow;
              end

              %if an offer was received, it will be processed first
              if offerManager{robotCount,1}.startOfferProcess
                  offerManager{robotCount,1}.startOfferProcess = false;
                  req = offerManager{robotCount,1}.offerList(1,1);
                  offerManager{robotCount,1}.offerList(:,1) = [];
                  offerManager{robotCount,1}.processOfferThread(node,req, robotCount); 
              end
              
              drawnow;
              %if the robot is not waiting for offerResult and has tasks
              %pending, the first task will be executed
              sizeTaskList = length(shared{robotCount,1}.acceptedTasks);
              if (offerManager{robotCount,1}.isWaitingForOfferResult == false) && (sizeTaskList ~= 0)

                  shared{robotCount,1}.currentExecutionTransportTask = shared{robotCount,1}.acceptedTasks(1,1);
                  shared{robotCount,1}.acceptedTasks(:,1) = [];
                  offerManager{robotCount,1}.executeTask(robotCount, node);
                  
                  %waits for offerResult, returns to waiting if no task
                  %left, otherwise executes next task
                  leave = false;
                  while ~leave
                      if offerManager{robotCount,1}.isWaitingForOfferResult  
                          if offerManager{robotCount,1}.startOfferProcess
                              offerManager{robotCount,1}.startOfferProcess = false;
                              req = offerManager{robotCount,1}.offerList(1,1);
                              offerManager{robotCount,1}.offerList(:,1) = [];
                              offerManager{robotCount,1}.processOfferThread(node,req, robotCount);
                          end
                      end
                      drawnow;
                      if offerManager{robotCount,1}.isWaitingForOfferResult || isempty(shared{robotCount,1}.acceptedTasks)
                          leave = true;
                          offerManager{robotCount,1}.isNotified = false;
                      else
                          shared{robotCount,1}.currentExecutionTransportTask = shared{robotCount,1}.acceptedTasks(1,1);
                          shared{robotCount,1}.acceptedTasks(:,1) = [];
                          offerManager{robotCount,1}.executeTask(robotCount, node);
                      end
                      drawnow;
                  end   
              else
                  %robot is waiting for offerResult
                  offerManager{robotCount,1}.isNotified = false;
              end
              drawnow;
              
          end

       end
      %calculates cost with and without new tasks, sends result back
      function processOfferThread(~,node,req,robotCount)
          global offerManager;
          global shared;
          global rai;

          executionTask = rosmessage('autonomous_robot/TransportTask');
          startPose = rosmessage('geometry_msgs/Pose2D');
          endPose = rosmessage('geometry_msgs/Pose2D');

          position = shared{robotCount,1}.currentPosition;
          startPosReached = shared{robotCount,1}.startPositionForTransportTaskReached;

          %assign value according to if robot is currently executing a task
          if shared{robotCount,1}.currentExecutionTransportTask.Id == -1
              startPose.X = shared{robotCount,1}.currentPosition.X;
              startPose.Y = shared{robotCount,1}.currentPosition.Y;
              endPose.X = shared{robotCount,1}.currentPosition.X;
              endPose.Y = shared{robotCount,1}.currentPosition.Y;
              executionTask.Id = -1;
          else
              startPose.X = shared{robotCount,1}.currentExecutionTransportTask.StartPoint.X;
              startPose.Y = shared{robotCount,1}.currentExecutionTransportTask.StartPoint.Y;
              endPose.X = shared{robotCount,1}.currentExecutionTransportTask.EndPoint.X;
              endPose.Y = shared{robotCount,1}.currentExecutionTransportTask.EndPoint.Y;
              executionTask.Id = shared{robotCount,1}.currentExecutionTransportTask.Id;
          end
          
          executionTask.EndPoint = endPose;
          executionTask.StartPoint = startPose;

          %calculate cost without new task
          tour = shared{robotCount,1}.acceptedTasks;
          tour = [executionTask tour];

          costDistanceWithoutNewTask = rai{robotCount,1}.calculateCostForTour(tour);
          costExecutionTimeWithoutNewTask = rai{robotCount,1}.calculateCostForTourExecutionTime(tour);

          if ~startPosReached
              if executionTask.Id ~= -1
                  costExecutionTimeWithoutNewTask = costExecutionTimeWithoutNewTask + abs(position.X-executionTask.StartPoint.X) + abs(position.Y-executionTask.StartPoint.Y);
              end
          end
          
          %calculate cost with new task
          rai{robotCount,1}.calculateOrderWithNewTask(req.Task,executionTask, robotCount);

          aproxTime = offerManager{robotCount,1}.executedTime + offerManager{robotCount,1}.costExecutionTime + abs(position.X-executionTask.StartPoint.X) + abs(position.Y-executionTask.StartPoint.Y);
          if ~startPosReached
              if executionTask.Id ~= -1
                  offerManager{robotCount,1}.costExecutionTime =  offerManager{robotCount,1}.costExecutionTime + abs(position.X-executionTask.StartPoint.X) + abs(position.Y-executionTask.StartPoint.Y);
              end
          end
          
          %send results to master
          tasknotsend = true;
          while tasknotsend
              try
                  tasknotsend = false;
                  ClientCounterOffer = ros.ServiceClient(node,'/ReceiveCounterOffer');
                  request = rosmessage(ClientCounterOffer);
                  request.From = req.To;
                  request.To = req.From;
                  request.Task = req.Task;
                  request.CostWithoutTask = costDistanceWithoutNewTask;
                  request.CostWithoutTaskExecutionTime = costExecutionTimeWithoutNewTask;
                  request.ApproximatedDistance = 0;
                  request.ApproximatedExecutionTime = aproxTime;
                  request.CostWithTask = offerManager{robotCount,1}.costDistance;
                  request.CostWithTaskExecutionTime = offerManager{robotCount,1}.costExecutionTime;
                  response = call(ClientCounterOffer,request,'Timeout',5);
                  drawnow;
              catch ME
                  tasknotsend = true;
                  msg = ['master did not receive counter offer']
              end
          end 
      end 
      
      %executes task by starting the simulation
      function executeTask(~,robotCount, node)
         global offerManager;
         global shared;
         shared{robotCount,1}.startPositionForTransportTaskReached = false;
         offerManager{robotCount,1}.simulation(robotCount, node);

         %after the task is executed, it's added to executedTasks
         offerManager{robotCount,1}.executedTasks = [offerManager{robotCount,1}.executedTasks shared{robotCount,1}.currentExecutionTransportTask];
         shared{robotCount,1}.currentExecutionTransportTask.Id = -1;
         shared{robotCount,1}.startPositionForTransportTaskReached = false;
         
         %send finished task to master
          tasknotsend = true;
          while tasknotsend
              try
                  tasknotsend = false;
                  FinishedTask = ros.ServiceClient(node,'/ReceiveFinishedTask');
                  request = rosmessage(FinishedTask);
                  request.Task = shared{robotCount,1}.currentExecutionTransportTask;
                  response = call(FinishedTask,request,'Timeout',5);
                  drawnow;
              catch ME
                  tasknotsend = true;
                  msg = ['master did not receive finished task']
              end
          end
      end
      
      %the simulation functions by creating a controller for a differential
      %drive model to follow a set of wayoints, afterwards the current
      %position of a roboter is reevalated inside a loop; only when the
      %robot is close to his goal the functions leaves the loop
      function simulation(~,robotCount, node)
          global shared;
          global offerManager;
          goalRadius = 0.1;
          
          %decides the model for the way a robot moves
          diffDrive = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
          
          %waypoints consists of the current position for a roboter, the
          %starting point of the current task and its ending point
          waypoints(1,1) = shared{robotCount,1}.currentPosition.X;
          waypoints(1,2) = shared{robotCount,1}.currentPosition.Y;
          waypoints(2,1) = shared{robotCount,1}.currentExecutionTransportTask.StartPoint.X;
          waypoints(2,2) = shared{robotCount,1}.currentExecutionTransportTask.StartPoint.Y;
          waypoints(3,1) = shared{robotCount,1}.currentExecutionTransportTask.EndPoint.X;
          waypoints(3,2) = shared{robotCount,1}.currentExecutionTransportTask.EndPoint.Y;

          robotInitialLocation = waypoints(1,:);
          robotGoal = waypoints(end,:);
          initialOrientation = 0;
          robotCurrentPose = [robotInitialLocation initialOrientation]';

          %the controller allows a differentialDriveModel to follow a set
          %of waypoints
          controller = controllerPurePursuit;
          controller.Waypoints = waypoints;
          controller.DesiredLinearVelocity = 0.6;
          controller.MaxAngularVelocity = 2;
          controller.LookaheadDistance = 0.3;
          
          %distanceToGoal allows to check if the robot has reached
          %its final destination and the simulation can be stopped
          distanceToGoal = norm(robotInitialLocation - robotGoal);
          
          %creates an object that allows to run loop at the same
          %frequency every time
          sampleTime = 0.1;
          vizRate = rateControl(1/sampleTime);
          frameSize = diffDrive.TrackWidth/0.8;
         
          figure(1);
          %until the robot has not reached its destination, the loop
          %continues to draw the figure
          while(distanceToGoal > goalRadius)
              offerManager{robotCount,1}.communication(robotCount, node);
              
              XPointReached = shared{robotCount,1}.currentPosition.X == shared{robotCount,1}.currentExecutionTransportTask.StartPoint.X;
              YPointReached = shared{robotCount,1}.currentPosition.X == shared{robotCount,1}.currentExecutionTransportTask.StartPoint.X;
              
              if XPointReached && YPointReached
                  shared{robotCount,1}.startPositionForTransportTaskReached = true;
              end
              
              %creates controller outputs that are used as inputs for
              %the robot to get its velocity
              [v, omega] = controller(robotCurrentPose);
              vel = derivative(diffDrive, robotCurrentPose, [v omega]);

              %update the current pose and get the new distanceToGoal
              %waypoints
              robotCurrentPose = robotCurrentPose + vel*sampleTime
              waypoints
              shared{robotCount,1}.currentPosition.X = robotCurrentPose(1,1);
              shared{robotCount,1}.currentPosition.Y = robotCurrentPose(2,1);
              distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

              %plots path 
              hold off
              plot(waypoints(:,1), waypoints(:,2),':');
              hold on

              %plots the actual robot
              plotTrVec = [robotCurrentPose(1:2); 0];
              plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
              plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
              %sets boundaries for the viewing window
              xlim([0 100]);
              ylim([0 100]);

              waitfor(vizRate);
          end
      
      end
      
      %after a simulation step the robot checks if it has been notified,
      %if there is a new task to process it begins the processOfferThread
      %before continuing the current task
      function communication(~, robotCount, node)
          global offerManager;
          drawnow;
          if offerManager{robotCount,1}.isNotified
              if offerManager{robotCount,1}.startOfferProcess
                  offerManager{robotCount,1}.startOfferProcess = false;
                  req = offerManager{robotCount,1}.offerList(1,1);
                  offerManager{robotCount,1}.offerList(:,1) = [];
                  offerManager{robotCount,1}.processOfferThread(node,req, robotCount); 
              end
          end
          
          offerManager{robotCount,1}.isNotified = false;

      end
    end
end