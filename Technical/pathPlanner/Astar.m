function [cameFrom] = astar()
% function [] = astar(goalx, goaly, startx, starty)
% Written by Bryce Hill
% 
% Purpose: _ 2-D space to search 
% 
% Inputs:
%   * goal (x,y)
%   * start (x,y)
% 
% Outputs:
%   * path

%% Define start and finish
sizex = 5;
sizey = 5;
goalx = 2.5;
goaly = 4;
startx = 1;
starty = 0;
hold on
plot(startx,starty,'o')
plot(goalx,goaly,'x')
%% Define operating space and generate obstacles
Space(sizex, sizey, startx, starty, goalx, goaly);
load('space.mat')   % Load vectors which define space and obstacles

%% Declare Closed and Open sets
closed = [];   % Closed is the set of nodes already evaluated
% open = node;   % Open is the set of nodes to be evaluated
length = numel(x);
openx = x;    % Set openx = single column array of x points
openy = y;    % Set openy = single column array of y points

%% Move to first node
buffer = [];
for i =1:1:length
    buffer(i) = distance(startx, starty, openx(i), openy(i));
end
[dum, index] = min(buffer);
cameFrom(1,1) = openx(index);
cameFrom(1,2) = openy(index);
plot(openx(index),openy(index),'o') 
xNode = openx(index); 
yNode = openy(index);       % semantics
closed = index;
        
step = 1;                   % Set up step tracking variable
noPath = 1;                         % Set flag for completeing a path
cameFrom
% Enter loop 
while((xNode ~= goalx || yNode ~= goaly) && noPath == 1) 
    [xNode, yNode, cameFrom, closed, index, step, noPath] = successor(xNode, yNode, openx, openy, cameFrom, length, neighbor, index, closed, goalx, goaly, step);
    cameFrom
end
cameFrom(step,1) = goalx;
cameFrom(step,2) = goaly;

plot(cameFrom(:,1),cameFrom(:,2),'--r')

end

