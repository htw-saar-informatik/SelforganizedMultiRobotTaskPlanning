load taskClient.mat

%connect to the ROS network
rosshutdown;
rosinit(masterURI);
%user Input
prompt = {'ID Auftrag:','Eingabe StartpositionX:','Eingabe StartpositionY:', 'Eingabe EndpositionX:', 'Eingabe EndpositionY:'};
dlgtitle = 'Input';
dims = [1 35];
definput = {'0','5','5', '10','10'};
answer = inputdlg(prompt,dlgtitle,dims,definput);

%creating node and client
node = {'/client_node_'};
numbertask = {int2str(counterTask)};
node1 = ros.Node(char(strcat(node,numbertask)));
counterTask = counterTask + 1;
save('taskClient.mat','counterTask','masterURI');
Client = ros.ServiceClient(node1,'/AddNewTransportTask');

request = rosmessage(Client);
request.Task.Id = str2double(answer(1,1));
request.Task.StartPoint.X = str2double(answer(2,1));
request.Task.StartPoint.Y = str2double(answer(3,1));
request.Task.EndPoint.X = str2double(answer(4,1));
request.Task.EndPoint.Y = str2double(answer(5,1));

%calling the AddNewTransportTask Service
response = call(Client,request,'Timeout',10);
