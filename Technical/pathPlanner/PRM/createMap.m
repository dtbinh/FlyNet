clc; clear; close all;

ftrows = 50;
ftcols = 25;
inrows = ftrows * 12;
incols = ftcols * 12;

map = zeros(inrows+1, incols+1);

obstacleNodes = [...
    5,    5;     5,    10;    17.5, 5;     12.5, 10;    12.5, 25;
    17.5, 20;    30,   0;     30,   5;     30,   10;    30,   17.5;
    30,   20;    35,   5;     35,   10;    35,   15;    40,   15;
    45,   10;    50,   15;    35,   25] * 12 + 1;

% Index '2' should always be bigger
nodeConnections = [...
    1  3;
    2  4;
    3  6;
    4  5;
    6  11;
    10 11;
    7  9;
    8  12;
    12 13;
    13 16;
    15 17;
    14 18];

for n = 1:size(nodeConnections,1)
    numRows            = abs(obstacleNodes(nodeConnections(n,1),1) - ...
                             obstacleNodes(nodeConnections(n,2),1)) + 1;
    numCols            = abs(obstacleNodes(nodeConnections(n,1),2) - ...
                             obstacleNodes(nodeConnections(n,2),2)) + 1;
    obstacleWalls      = zeros(max(numRows, numCols), 2);
    obstacleWalls(:,1) = obstacleNodes(nodeConnections(n,1),1):...
                         obstacleNodes(nodeConnections(n,2),1);
    obstacleWalls(:,2) = obstacleNodes(nodeConnections(n,1),2):...
                         obstacleNodes(nodeConnections(n,2),2);
    obstacleInd        = sub2ind(size(map), obstacleWalls(:,1), obstacleWalls(:,2));
    map(obstacleInd)   = 1;
end

map = flipud(map);
imagesc(map)
save('map.mat', 'map')

%%
% Now we create an instance of a robot with the PRM navigation algorithm
prm = PRM(map);

% and because PRM is a probabilistic method we will reset the random number
% generator to a known state
randinit

% Now we define the goal and start coordinates
goal  = [22.5,40]*12;
start = [12.5,2.5]*12;

% then ask the robot to plan a path to goal (it will take few seconds)
prm.plan();

% The roadmap planner does not need to know, yet, the goal or start positions, it
% chooses random points in the world and tries to find obstacle-free paths between
% them (like railway lines or freeways)

% Now we can display the obstacles and the cost to reach the goal from every
% point in the world
prm.plot();

% Now we can execute the planned path, it will be animated with green dots
prm.path(start, goal)

%%
setpoints = unique(reshape(prm.localSp, [2,length(prm.localSp)/2])', 'rows', 'stable');
path.x = timeseries(setpoints(:,1),(0:8)*60);
path.y = timeseries(setpoints(:,2),(0:8)*60);
path.z = timeseries(3*ones(9,1),(0:8)*60);
path.psi = timeseries(zeros(9,1),(0:8)*60);
save('pathFlemming.mat', 'path');
