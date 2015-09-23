%% Hokuyo Data Processing
%
% This file reads in a csv file saved by running the python script
% "hokuyo_data.py" 
% Hokuyo range data is plotted and processed to find sensor biases.
%% User Input:
close all; clear all; clc; 
format compact

visualize = 1;



%% Processing:

% Get Hokuyo Settings:
setting = csvread('hokuyo_data.txt',1,0,[1,0,1,7]);
time_start = setting(1);
ang_min = setting(2);
ang_max = setting(3);
ang_inc = setting(4);
t_scan = setting(5);
range_min = setting(6);
range_max = setting(7);
intensities = setting(8);

% Get Hokuyo Ranges:
data = csvread('hokuyo_data.txt',4,0);
time = data(:,1);
ranges = data(:,2:end);

ang = linspace(ang_min+pi/2,ang_max+pi/2,size(ranges,2));

if visualize == 1
    figure
    % Plot results:
    for m = 1:size(ranges,1)
        for n = 1:size(ranges,2)
            dist(n) = ranges(m,n);
        end
        [x,y] = pol2cart(ang,dist);
        plot(x,y)
        axis([-6,6,-6,6])
        grid
        pause(0.0001)
    %     pause
    end
end

% Get variance for each angle:
var = var(ranges);
max_var = max(var)
total_inf = sum(sum(ranges == Inf))
total_nan = sum(sum(isnan(ranges)))
