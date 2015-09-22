%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller Tuning Post Processing
% Project DVZ
% Programmer: Mark Sakaguchi
% Created: 2/20/2015
% Updated: 2/23/2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clean Up
clear all
close all
clc

%% Add functions and data paths
addpath('parse_functions');
addpath('plot_functions');
% addpath('Position_Controller_Tuning_03_03_15');
%addpath('px4flow_tuning_03_19_15');
addpath('Collision_Avoidance_04_03_15');

%% Load data
test_num = 12;

 file_alt = ['alt_test',num2str(test_num),'.txt'];
% file_vel = ['vel_test',num2str(test_num),'.txt'];
% file_pos = ['pos_test',num2str(test_num),'.txt'];
% file_rc = ['rc_test',num2str(test_num),'.txt'];

 alt_cont = parseAltController(file_alt);
% vel_cont = parseVelocityController(file_vel);
% pos_cont = parsePositionController(file_pos);
% rc = parseRC(file_rc);

%% Plot data
 plotAltController(alt_cont);
% plotVelocityController(vel_cont);
% plotPositionController(pos_cont);
% plotRC(rc);
return

%% Plot Altitude Stuff
test_num = 2;

file_flow = ['flow_test', num2str(test_num),'.txt'];
file_alt = ['alt_test', num2str(test_num),'.txt'];

flow = parseFlow(file_flow);
alt = parseAltController(file_alt);

plotAltController(alt)
plotFlow(alt,flow)
