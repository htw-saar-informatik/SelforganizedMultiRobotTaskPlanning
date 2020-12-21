%clearing workspace and loading saved values from Parameter.mat 
clear
load Parameter.mat
%figure(1);

%collect user input for number of robots and tasks
resultInput = Input();
anzahlRob = resultInput(1,1);
anzahlAuftraege = resultInput(1,2);
costPerformance = resultInput(1,3);

%create random values for robots and tasks (id and positions)
robotList = randomRobots(anzahlRob);
taskList = randomTasks(anzahlAuftraege);

%ROS-Network (shutdown first, so no error thrown if the network is still running -> try catch solution)
rosshutdown;
rosinit;

%delete the currently runnig pool of workers and start a new one with the
%same amount of workers as robots
%delete(gcp('nocreate'));
%parpool('local',anzahlRob);

%each worker creates own ROS-Node and starts the robot function
parfor i=1:anzahlRob
    rosshutdown;
    rosinit(masterURI);
    
    if i <= anzahlRob
        robot(robotList,taskList,i,anzahlRob,anzahlAuftraege,costPerformance); 
    end
end

function result = Input()
    prompt = {'Eingabe Anzahl der Roboter (2,4,8,12,16):','Eingabe Anzahl Aufträge (10,20,50,100,200,300):', 'Ziel der Optimierung(0=Zeit, 1=Distanz):'};
    dlgtitle = 'Input';
    dims = [1 35];
    definput = {'2','10','0'};
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    anzahlRob = str2double(answer(1,1));
    anzahlAuftraege = str2double(answer(2,1));
    costPerformance = str2double(answer(3,1));
    
    result = [anzahlRob, anzahlAuftraege, costPerformance];
end
function result = randomRobots(anzahlRob)
    %zufälliger Roboterwert und Startposition
    r = randi([5,15],anzahlRob, 3);
    for c = 1:anzahlRob
        robotList(c, 1) = r(c, 1);
        robotList(c, 2:3) = r(c, 2:3);
    end
    result = robotList;
end
function result = randomTasks(anzahlAuftraege)
    %zufällige Tasks
    r = randi([5,15],anzahlAuftraege, 5);
    for c = 1:anzahlAuftraege
        taskList(c, 1) = r(c, 1);
        taskList(c, 2:3) = r(c, 2:3);
        taskList(c, 4:5) = r(c, 4:5);
    end
    result = taskList;
end

%robot function that creates services and calls the class functions
function robot(Robotlist,Tasklist,robotCount,anzahlRob,anzahlAuftraege,costPerformance)
    %class objects saved in global cells so every robot has their own
    %object and can access individual values
    global offerManager;
    offerManager = {};
    global distributer;
    distributer = {};
    global shared;
    shared{robotCount,1} = SharedFunctions;
    global rai;
    rai = {};
    
    %first robot is masterNode
    isMasterNode = false;
    if robotCount == 1
        isMasterNode = true;
    end
    
    %nodeID as the ROS-Node name
    shared{robotCount,1}.nodeID = Robotlist(robotCount,1);
    node = {'/node_'};
    number = int2str(shared{robotCount,1}.nodeID);
    name = char(strcat(node,number));
    node = ros.Node(name);
    
    shared{robotCount,1}.frame_id = Robotlist(robotCount,1);
    shared{robotCount,1}.currentPosition.X = Robotlist(robotCount,2);
    shared{robotCount,1}.currentPosition.Y = Robotlist(robotCount,3);
    shared{robotCount,1}.currentExecutionTransportTask.Id = -1;
    shared{robotCount,1}.countRobots = anzahlRob;
    shared{robotCount,1}.countTasksForClientAutomation = anzahlAuftraege;
    shared{robotCount,1}.distributionMode = costPerformance;

    %every robot creates the needed ROS-Services for the communication to
    %work and then starts their process
    if isMasterNode  
        distributer{robotCount,1} = TaskDistributer;
        
        %the masterNode receives the randomly created list of tasks
        transportTask = [];
        for count =1:length(Tasklist(:,1))
            transportTask = [transportTask rosmessage('autonomous_robot/TransportTask')];
        end
        for count =1:length(Tasklist(:,1))
            transportTask(count).Id = 0 + count;
            transportTask(count).StartPoint.X = Tasklist(count,2);
            transportTask(count).StartPoint.Y = Tasklist(count,3);
            transportTask(count).EndPoint.X = Tasklist(count,4);
            transportTask(count).EndPoint.Y = Tasklist(count,5);
            distributer{robotCount,1}.transportTasks = [distributer{robotCount,1}.transportTasks transportTask(count)];
        end

        distributer{robotCount,1}.executionTimeForRobots = [];
        distributer{robotCount,1}.offerResults = [];
        distributer{robotCount,1}.finishedTasks = [];
        Robotlist(robotCount,:) = [];
        shared{robotCount,1}.robotNames = Robotlist;

        %create ROS-Server to receive new task and counteroffer
        ServerAddNewTransportTask = createServerAddNewTransportTask(node, distributer{robotCount,1}, robotCount);
        ServerReceiveCounterOffer = createServerReceiveCounterOffer(node, distributer{robotCount,1}, robotCount);
        ServerReceiveFinishedTask = createServerReceiveFinishedTask(node, distributer{robotCount,1}, robotCount);

        %receives new tasks, sends them to other robots, calculates the
        %result and informs other robots of their win or loss
        distributer{robotCount,1}.startTaskProcessing(robotCount, node, anzahlRob, anzahlAuftraege, costPerformance);
    else
        offerManager{robotCount,1} = OfferManager;
        shared{robotCount,1}.robotNames = Robotlist(1,1);
        
        %create ROS-Server to receive offer and be informed if won or lost
        ServerReceiveOffer = createServerReceiveOffer(node, number, offerManager{robotCount,1}, robotCount);
        ServerWinOffer = createServerWinOffer(node, number, offerManager{robotCount,1}, shared{robotCount,1},robotCount);
        ServiceLooseOffer = createServiceLooseOffer(node, number, offerManager{robotCount,1}, shared{robotCount,1},robotCount);

        %receives offers, calculates RAI and sends result back,
        %executes all given tasks
        offerManager{robotCount,1}.startTaskProcessing(node, robotCount);
    end
end

%ROS-Server creation
function server = createServerAddNewTransportTask(node,  distributer, robotCount)
    server = ros.ServiceServer(node,'/AddNewTransportTask','autonomous_robot/AddNewTransportTask',{@distributer.callbackAddNewTransportTask,robotCount});
end
function server = createServerReceiveCounterOffer(node, distributer, robotCount)
    server = ros.ServiceServer(node,'/ReceiveCounterOffer','autonomous_robot/ReceiveCounterOffer',{@distributer.callbackReceiveCounterOffer,robotCount});
end
function server = createServerReceiveFinishedTask(node,  distributer, robotCount)
    server = ros.ServiceServer(node,'/ReceiveFinishedTask','autonomous_robot/ReceiveFinishedTask',{@distributer.callbackReceiveFinishedTask,robotCount});
end
function server = createServerReceiveOffer(node, number, offerManager, robotCount)
    name = char(strcat('/ReceiveOffer',number));
    server = ros.ServiceServer(node,name,'autonomous_robot/ReceiveOffer',{@offerManager.callbackReceiveOffer, robotCount});
end
function server = createServerWinOffer(node, number, offerManager, shared, robotCount)
    name = char(strcat('/WinOffer',number));
    server = ros.ServiceServer(node,name,'std_srvs/Empty',{@offerManager.callbackWinOffer, shared, robotCount});
end
function server = createServiceLooseOffer(node, number, offerManager, shared, robotCount)
    name = char(strcat('/LooseOffer',number));
    server = ros.ServiceServer(node,name,'std_srvs/Empty',{@offerManager.callbackLooseOffer, shared, robotCount});
end