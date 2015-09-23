% Battery Tests Post Processing
% Project DVZ
% Programmer: Mark Sakaguchi
% Created: 2/8/2015
% Updated: 2/8/2015

clear all
close all
clc

format long

addpath('02_08_15');

%% Load data
filename1 = 'alt_pitch1.txt';
filename2 = 'batt_pitch1.txt';
filename3 = 'rc_pitch1.txt';
alt1 = parseAltController(filename1);
batt1 = parseBattery(filename2);
rc1 = parseRC(filename3);
alt1.time = alt1.time - alt1.time(1);
batt1.time = batt1.time - batt1.time(1);
rc1.time = rc1.time - rc1.time(1);

filename1 = 'alt_pitch2.txt';
filename2 = 'batt_pitch2.txt';
filename3 = 'rc_pitch2.txt';
alt2 = parseAltController(filename1);
batt2 = parseBattery(filename2);
rc2 = parseRC(filename3);
alt2.time = alt2.time - alt2.time(1);
batt2.time = batt2.time - batt2.time(1);
rc2.time = rc2.time - rc2.time(1);

%% Form variables to be used in SYSID toolbox for pitch

time1 = rc1.time;
theta_in1 = rc1.pitch;
theta_out1 = alt1.theta;

time2 = rc2.time;
theta_in2 = rc2.pitch;
theta_out2 = alt2.theta;

ident
%% Plot data
% plotAltController(alt1)
% plotBattery(batt)
% plotRC(rc1)
% plotAttitude(att)