clc; clear; close all;

% Load map
load('map.mat');

numNodes = 100:200;
tproc    = zeros(1, length(numNodes));
    
% Define the goal and start coordinates
goal  = [22.5,40]*12;
start = [12.5,2.5]*12;

for n = 1:length(numNodes)
    
    % Start counting processing time
    tstart = tic;
    
    % Create PRM object
    prm = PRM(map);
    
    % Initialize random number to known state
    randinit
    
    % Redefine number of points
    prm.npoints = numNodes(n);
    
    % Plan a path to goal
    prm.plan();
    
    % Stop counting processing time
    tproc(n) = toc(tstart);
end

%%

figure(2)
hold on
grid on
plot(numNodes, tproc, 'ro', 'markersize', 6);
title('Processing Time vs Number of Nodes');
ylabel('Processing Time [s]'); xlabel('Number of Nodes');

polycoeffs = polyfit(numNodes, tproc, 2);
linreg = polyval(polycoeffs, numNodes);
plot(numNodes, linreg, 'b', 'linewidth', 2);
text(150,1.5,'y = 0.00012x^2 - 0.006x + 0.395',...
     'HorizontalAlignment','left',...
     'VerticalAlignment','top',...
     'BackgroundColor',[1 1 1],...
     'EdgeColor', [0 0 0],...
     'FontSize',10);


