clc; clear; close all;
profile on
%% NEEDS rvctools IN MATLAB PATH

% Dimensions of Flemming space [ft/in]
ftrows = 50;
ftcols = 25;
inrows = ftrows * 12;
incols = ftcols * 12;

%  Initialize map with zeros
maxrows = inrows + 1;
maxcols = incols + 1;
map = zeros(maxrows, maxcols);

% Locations of wall corner nodes
obstacleNodes = 1 + 12 * [...
    5,    5;     5,    10;    17.5, 5;
    12.5, 10;    12.5, 25;    17.5, 20;
    30,   0;     30,   5;     30,   10;
    30,   17.5;  30,   20;    35,   5;
    35,   10;    35,   15;    40,   15;
    45,   10;    50,   15;    35,   25;
    0,    0;     0,    25;    50,   0;
    50,   25];

% Connections between wall corner nodes
% note: index '2' should always be bigger
nodeConnections = [...
    1  3;     2  4;     3  6;     4  5;
    6  11;    10 11;    7  9;     8  12;
    12 13;    13 16;    15 17;    14 18;
    19 20;    19 21;    20 22;    21 22];

for n = 1:size(nodeConnections,1)
    
    % Determine how many row/col indices will be needed to make connection
    numRows            = abs(obstacleNodes(nodeConnections(n,1),1) - ...
                             obstacleNodes(nodeConnections(n,2),1)) + 1;
    numCols            = abs(obstacleNodes(nodeConnections(n,1),2) - ...
                             obstacleNodes(nodeConnections(n,2),2)) + 1;
    obstacleWalls      = zeros(max(numRows, numCols), 2);
    
    % Set wall row/col indices to '1' in the map
    obstacleWalls(:,1) = obstacleNodes(nodeConnections(n,1),1):...
                         obstacleNodes(nodeConnections(n,2),1);
    obstacleWalls(:,2) = obstacleNodes(nodeConnections(n,1),2):...
                         obstacleNodes(nodeConnections(n,2),2);
    obstacleInd        = sub2ind(size(map), ...
                         obstacleWalls(:,1), ...
                         obstacleWalls(:,2));
    map(obstacleInd)   = 1;
    
end

% Add padding to walls as a safety factor, default '6' inches on each side
padval = 6;
[r, c] = ind2sub(size(map), find(map == 1));
for i = 1:length(r)
    padrows = r(i) - padval:r(i) + padval;
    padcols = c(i) - padval:c(i) + padval;
    
    if max(padrows) > maxrows
        padrows = r(i) - padval:maxrows;
    end
    if min(padrows) < 1
        padrows = 1:r(i) + padval;
    end
    if max(padcols) > maxcols
        padcols = c(i) - padval:maxcols;
    end
    if min(padcols) < 1
        padcols = 1:c(i) + padval;
    end
    
    map(padrows,padcols) = 1;
end

% Save map
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

%% Create a path for running Quad dynamics model

% Time [sec] when each position set point command should be given
timeline = [0 6 7 7.5 8 8.5 9 9.5 10] * 60;

% Prepare position set point format for model
setPts   = unique(reshape(prm.localSp, [2,length(prm.localSp)/2])', ...
           'rows', 'stable');
% Create struct of position/yaw set point commands
path.x   = timeseries(setPts(:,1),timeline);
path.y   = timeseries(setPts(:,2),timeline);
path.z   = timeseries(36*ones(9,1),timeline);
path.psi = timeseries(zeros(9,1),timeline);
save('pathFlemming.mat', 'path');

profile viewer


