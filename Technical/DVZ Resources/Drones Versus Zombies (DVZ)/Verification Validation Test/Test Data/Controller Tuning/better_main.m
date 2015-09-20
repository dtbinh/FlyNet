%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller Tuning Post Processing
% Project DVZ
% Programmer: Mark Sakaguchi
% Created: 2/20/2015
% Updated: 2/23/2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Cleansing
clear all
close all
clc

%% Add functions and data paths
addpath('parse_functions');
addpath('plot_functions');
addpath('Position_Controller_Tuning_03_02_15');
addpath('Sysid_tests_03_02_15')
%% Load data
 test_num = 1;
% 
% % file_alt = ['alt_test',num2str(test_num),'.txt'];
% %file_roll = ['roll_test',num2str(test_num),'.txt'];
% %file_pitch = ['pitch_test',num2str(test_num),'.txt'];
%  file_vel = ['vel_test',num2str(test_num),'.txt'];
% file_pos = ['pos_test',num2str(test_num),'.txt'];
% %file_rc = ['rc_test',num2str(test_num),'.txt'];
% 
% % alt_cont = parseAltController(file_alt);
% %roll_cont = parseRollController(file_roll);
% %pitch_cont = parsePitchController(file_pitch);
%  vel_cont = parseVelocityController(file_vel);
% pos_cont = parsePositionController(file_pos);
% %rc = parseRC(file_rc);
% 
% %% Plot data
% %plotAltController(alt_cont);
% %plotRollController(roll_cont);
% %plotPitchController(pitch_cont);
% plotVelocityController(vel_cont);
% plotPositionController(pos_cont);
% %plotRC(rc);

%% Plot SYSID Info
pitch_roll_str = 'pitch';
for sys_test = 1:6;

filename = ['sys_' pitch_roll_str '_' num2str(sys_test) '.txt'];
sys_data(sys_test) = parseSYSID(filename);
t_samp(sys_test) = mean(diff(sys_data(sys_test).time));
plotSYSID(sys_data(sys_test))

end

t_samp_ave = mean(t_samp);