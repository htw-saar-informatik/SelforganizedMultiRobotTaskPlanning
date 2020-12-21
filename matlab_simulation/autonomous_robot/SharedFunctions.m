classdef SharedFunctions
    %SharedFunctions includes properties and functions that both master and
    %the other robots use
   properties 
        robotNames
        countRobots int32 = (0)
        nodeID int32 = (0)
        frame_id int32 = (-1)
        distributionMode int32 = (0)
        currentPosition = rosmessage('geometry_msgs/Pose2D')
        countTasksForClientAutomation int32 = (0)
        startPositionForTransportTaskReached int32 = (0)
        currentExecutionTransportTask = rosmessage('autonomous_robot/TransportTask') 
        acceptedTasks = []
        calculatedOrderForNewTask = []
   end
   properties (Access = private)
        idCounter int32 = (0)
   end
   methods
       %when a new task arrives its id is created with this function
       function result = getNextId(obj)
           obj.idCounter = obj.idCounter + 1;
           result = obj.idCounter;
       end
       
       function result = getRandomId(~)
           result = randi([5,95],1, 1);
       end
   end
end