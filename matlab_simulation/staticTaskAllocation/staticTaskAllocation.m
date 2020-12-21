%clean workspace and load emptyMap and robotRadius
clear
load Parameter.mat;

%get User Input for number of robots, tasks, roundsOptimization and
%optimization goal
resultInput = Input();
anzahlRob = resultInput(1,1);
anzahlAuftraege = resultInput(1,2);
roundsOptimization = resultInput(1,3);
costPerformance = resultInput(1,4);

%create random values for robots and tasks (id and positions)
robotList = randomRobots(anzahlRob);
taskList = randomTasks(anzahlAuftraege);

%start Timer
tic;

%start RAI by creating matrix first, then starting the algorithm
rai = RAI;
matrix = rai.createMatrix(anzahlRob, anzahlAuftraege, robotList, taskList);
RAIResult = rai.startAlgo(anzahlRob, anzahlAuftraege, matrix, costPerformance,  taskList, roundsOptimization);
tour = RAIResult(3:end)

%start simulation by creating tours for each individual robot
simulation = Simulation;
robotTours = simulation.createTours(tour, anzahlRob, robotList, taskList);
robotTours
for i=1:anzahlRob
    simulation.createModel(i, robotTours);
end

simulation.simulate(anzahlRob);

%collect output in csv-file
Output(anzahlRob, anzahlAuftraege, costPerformance, roundsOptimization, RAIResult);

function result = Input()
    prompt = {'Eingabe Anzahl der Roboter (2,4,8,12,16):','Eingabe Anzahl Aufträge (10,20,50,100,200,300):', 'Eingabe Anzahl an Optimierungsrunden(50,100,200,400):', 'Ziel der Optimierung(0=Zeit, 1=Distanz):'};
    dlgtitle = 'Input';
    dims = [1 35];
    definput = {'2','10','50','0'};
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    anzahlRob = str2double(answer(1,1));
    anzahlAuftraege = str2double(answer(2,1));
    roundsOptimization = str2double(answer(3,1));
    costPerformance = str2double(answer(4,1));
    
    result = [anzahlRob, anzahlAuftraege, roundsOptimization, costPerformance];
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

function Output(anzahlRob, anzahlAuftraege, costPerformance, roundsOptimization, RAIResult)
    filename = [num2str(anzahlRob),'roboter',num2str(anzahlAuftraege),'task',num2str(costPerformance),'goal',num2str(roundsOptimization),'rounds','.csv'];
    %end Timer
    elapsedTime = toc;
    data = [RAIResult(1) RAIResult(2) elapsedTime];
    writematrix(data,filename,'WriteMode','append'); 
end