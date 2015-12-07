function [cameFrom] = Astar()
% Written by Bryce Hill
% 
% Purpose: _ 2-D space to search 
% 
% Inputs:
% 
% Outputs:
%   * path

close all
tic
%% Define start and finish
sizex = 20;
sizey = 20;
goalx = 25;% 12
goaly = 55;% 18
startx = 2;
starty = 4;
term = 3;   % termination radius limit
margin = 2; % distance allowed between vehicle and obstacles

hold on
plot(startx,starty,'o')
plot(goalx,goaly,'x')
%% Define operating space and generate obstacles
Space(sizex, sizey, startx, starty, goalx, goaly, margin);
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
% plot(openx(index),openy(index),'o') 
xNode = openx(index); 
yNode = openy(index);       % semantics
closed = index;
lastx = [startx xNode];
lasty = [starty yNode];

step = 1;                   % Set up step tracking variable
noPath = 1;                 % Set flag for completeing a path
index;
cameFrom;
% Enter loop 
while((xNode ~= goalx || yNode ~= goaly) && noPath == 1) 
    [xNode, yNode, cameFrom, closed, index, step, noPath] = successor(xNode, yNode, openx, openy, cameFrom, length, neighbor, index, closed, goalx, goaly, step,term);
    cameFrom;
end
toc

cameFrom(step,1) = goalx;
cameFrom(step,2) = goaly;
hold on
plot(cameFrom(:,1),cameFrom(:,2),'r')
plot(lastx, lasty, '--r')

load('mapt.mat')
plot(mapt(:,1),mapt(:,2),'k')
hold on
plot([6 12],[55,45],'w')
plot([18; 30],[13,18],'w') 
plot([20 12],[25,25],'w')
end

