classdef Simulation
    %Simulation needs to create a tour for each individual robot, then
    %create a model for each of them;
    %afterwards the simulation can be started
    
    methods
        %sorts the tour in a way that makes it easier to assign waypoints
        %later for each individual robot
        function result = createTours(~, tour, anzahlRob, robotList, taskList)
            while tour(1,1) > anzahlRob
                tour = [tour(1,2:end) tour(1,1)]; 
            end
            tourPosition = [];
            counter = 1;
            %creates a cell including all robots and tasks
            for element = tour
                if element <= anzahlRob
                    tourPosition(counter,1) = element;
                    tourPosition(counter,2) = robotList(element, 2);
                    tourPosition(counter,3) = robotList(element, 3);
                end
                if element > anzahlRob
                    tourPosition(counter,1) = element;
                    tourPosition(counter,2) = taskList(element - anzahlRob, 2);
                    tourPosition(counter,3) = taskList(element - anzahlRob, 3);
                    tourPosition(counter,4) = taskList(element - anzahlRob, 4);
                    tourPosition(counter,5) = taskList(element - anzahlRob, 5);
                end
                counter = counter +1;
            end
            
            RobotTours = [];
            countTask = 1;
            countForTour = 1;
            
            %creates a cell where each robot gets a new row with its tasks
            %listed in the same row
            for i=1:length(tourPosition(:,1))
                if tourPosition(i,1) <= anzahlRob
                    if i == 1
                    countTask = 1;

                    RobotTours(countForTour, countTask) = tourPosition(i, 2);
                    RobotTours(countForTour, countTask+1) = tourPosition(i, 3);
                    countTask = countTask +2;
                    else
                    countForTour = countForTour +1;
                    countTask = 1;

                    RobotTours(countForTour, countTask) = tourPosition(i, 2);
                    RobotTours(countForTour, countTask+1) = tourPosition(i, 3);
                    countTask = countTask +2;
                    end
                end
                if tourPosition(i,1) > anzahlRob
                    RobotTours(countForTour, countTask) = tourPosition(i, 2);
                    RobotTours(countForTour, countTask+1) = tourPosition(i, 3);
                    RobotTours(countForTour, countTask+2) = tourPosition(i, 4);
                    RobotTours(countForTour, countTask+3) = tourPosition(i, 5);

                    countTask = countTask +4;
                end
            end
            result = RobotTours;
        end
        %creates a model and controller of that model for each robot and assigns the waypoints
        function createModel(~, i, robotTours)
            %make variables global so the simulation function can use them
            global diffDrive;
            global waypoints;
            global robotInitialLocation;
            global robotGoal;
            global initialOrientation;
            global robotCurrentPose;
            global controller;
            global goalRadius;
            
            goalRadius = 0.1;
            
            %decides the model for the way a robot moves
            diffDrive{i} = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
            waypoints{i} = robotTours(i,1:end);
            
            %waypoints have to be rearranged for the controller
            newwaypoints = [];
            for count =1:length(waypoints{i})/2
                newwaypoints = [newwaypoints; waypoints{i}(1,1:2)];
                waypoints{i}(:,1:2) = [];
            end

            waypoints{i} = newwaypoints;
            robotInitialLocation{i} = newwaypoints(1,1:2);
            robotGoal{i} = newwaypoints(end,1:2);
            initialOrientation{i} = 0;
            robotCurrentPose{i} = [robotInitialLocation{i} initialOrientation{i}]';
            
            %the controller allows a differentialDriveModel to follow a set
            %of waypoints
            controller{i} = controllerPurePursuit;
            controller{i}.Waypoints = waypoints{i};
            controller{i}.DesiredLinearVelocity = 0.6;
            controller{i}.MaxAngularVelocity = 2;
            controller{i}.LookaheadDistance = 0.3;
        end
        
        %simulates all robot models
        function simulate(obj, anzahlRob)
            global diffDrive;
            global waypoints;
            global robotInitialLocation;
            global robotGoal;
            global robotCurrentPose;
            global controller;
            global goalRadius;
            global distanceToGoal;
            notReachedRobots = [];
            
            %distanceToGoal allows to check if the robot has reached
            %its final destination and the simulation can be stopped
            for count = 1:anzahlRob
                distanceToGoal{count} = norm(robotInitialLocation{count} - robotGoal{count});
                frameSize = diffDrive{count}.TrackWidth/0.8;
                notReachedRobots = [notReachedRobots count];
            end

            %creates an object that allows to run loop at the same
            %frequency every time
            sampleTime = 0.1;
            vizRate = rateControl(1/sampleTime);
            figure
            
            %until the robots have not reached their destination, the loop
            %continues to draw the figure
            while(~isempty(notReachedRobots))
                
                notReachedRobots = obj.checkIfAllReached(goalRadius);
    
                %information about their position and the distanceToGoal is
                %reevaluated for each robot
                for i=notReachedRobots
                    %creates controller outputs that are used as inputs for
                    %the robot to get its velocity
                    [v, omega] = controller{i}(robotCurrentPose{i});
                    vel = derivative(diffDrive{i}, robotCurrentPose{i}, [v omega]);

                    %update the current pose and get the new distanceToGoal
                    robotCurrentPose{i} = robotCurrentPose{i} + vel*sampleTime; 
                    distanceToGoal{i} = norm(robotCurrentPose{i}(1:2) - robotGoal{i}(:));
                end
                               
                %plots path for the last robot
                hold off
                for i=notReachedRobots
                    plot(waypoints{i}(:,1), waypoints{i}(:,2),':')
                end
                hold on

                %plots the actual robot
                for i=notReachedRobots
                    plotTrVec = [robotCurrentPose{i}(1:2); 0];
                    plotRot = axang2quat([0 0 1 robotCurrentPose{i}(3)]);
                    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
                end
                %sets boundaries for the viewing window
                xlim([0 100])
                ylim([0 100])

                waitfor(vizRate);
            end
            
        end
        
        %function checks if all robots have reached their destination
        function result = checkIfAllReached(~, goalRadius)
            global distanceToGoal;
            result = [];
            for i=1:length(distanceToGoal)
                if distanceToGoal{i} > goalRadius
                    result = [result i];
                end
            end

        end
    end
end

