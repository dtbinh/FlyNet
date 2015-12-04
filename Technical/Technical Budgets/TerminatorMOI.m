% Tyler Clayton
% Terminator Quad Moments of Inertia Calculation
% TerminatorMOI.m
% UPDATED 12/2/2015

clear all; close all; clc;

%% define element dimensions
% for rectangular elements dimensions are (a,b,L) 
% a-sizes x 
% b-sizes y
% L-sizes z
in2m = 0.0254;         % inches to meters

% Arm -  includes esc x4
arm.m = (24.5+28.4)/1000;
arm.a = 5.5*in2m;
arm.b = 1*in2m;
arm.L = 0.25*in2m;
arm.rx = 4.25*in2m;
arm.ry = 0;
arm.rz = arm.rx;
arm.num = 4;
arm.Ix = arm.m/12*(arm.a^2+arm.L^2);
arm.Iy = arm.m/12*(arm.a^2+arm.L^2);
arm.Iz = arm.m/12*(arm.a^2+arm.L^2);

% Battery x1
batt.m = 815.8/1000;
batt.a = 2.5*in2m;
batt.b = 6.5*in2m;
batt.L = 1.5*in2m;
batt.rx = 1.875*in2m;
batt.ry = arm.rx;
batt.rz = 0;
batt.num = 1;
batt.Ix = batt.m/12*(batt.a^2+batt.L^2);
batt.Iy = batt.m/12*(batt.a^2+batt.L^2);
batt.Iz = batt.m/12*(batt.a^2+batt.L^2);

% Center body-  includes rest of frame, pixhawk, odroid x1
centbod.m = (427.35-4*24.5)/1000;
centbod.a = 3.5*in2m;
centbod.b = 14*in2m;
centbod.L = 2.25*in2m;
centbod.rx = 0;
centbod.ry = 0;
centbod.rz = 0;
centbod.num =1;
centbod.Ix = centbod.m/12*(centbod.a^2+centbod.L^2);
centbod.Iy = centbod.m/12*(centbod.a^2+centbod.L^2);
centbod.Iz = centbod.m/12*(centbod.a^2+centbod.L^2);

% guidance -  includes landing gear x1
guid.m = (90+462.2)/1000;
guid.a = 6*in2m;
guid.b = 6*in2m;
guid.L = 2*in2m;
guid.rx = 2.125*in2m;
guid.ry = guid.rx;
guid.rz = 0;
guid.num = 1;
guid.Ix = guid.m/12*(guid.a^2+guid.L^2);
guid.Iy = guid.m/12*(guid.a^2+guid.L^2);
guid.Iz = guid.m/12*(guid.a^2+guid.L^2);

% motor - includes rotor weight x4
motor.m = (11.1+68.7)/1000;
motor.r = (1.125/2)*in2m;
motor.h = 1.5*in2m;
motor.rx = 7*in2m;
motor.ry = 5.25*in2m;
motor.rz = sqrt(motor.rx^2+motor.ry^2);
motor.num = 4;
motor.Ix = motor.m/12*(3*motor.r^2+motor.h^2);
motor.Iy = motor.m/12*(3*motor.r^2+motor.h^2);
motor.Iz = .5*motor.m*motor.r^2;

% Prop guards x2
propgd.m = 39/1000;
propgd.a = .5*in2m;
propgd.b = 34*in2m;
propgd.L = .5*in2m;
propgd.rx = 13*in2m;
propgd.ry = 0;
propgd.rz = propgd.rx;
propgd.num = 2;
propgd.Ix = propgd.m/12*(propgd.a^2+propgd.L^2);
propgd.Iy = propgd.m/12*(propgd.a^2+propgd.L^2);
propgd.Iz = propgd.m/12*(propgd.a^2+propgd.L^2);

%% Total MOI using Parrallel Axis Thereom
TermMOI.Ix = arm.num*(arm.Ix+arm.m*arm.rx^2)+batt.num*(batt.Ix+batt.m*batt.rx^2)+centbod.num*(centbod.Ix+centbod.m*centbod.rx^2)+guid.num*(guid.Ix+guid.m*guid.rx^2)+motor.num*(motor.Ix+motor.m*motor.rx^2)+propgd.num*(propgd.Ix+propgd.m*propgd.rx^2);
TermMOI.Iy = arm.num*(arm.Iy+arm.m*arm.ry^2)+batt.num*(batt.Iy+batt.m*batt.ry^2)+centbod.num*(centbod.Iy+centbod.m*centbod.ry^2)+guid.num*(guid.Iy+guid.m*guid.ry^2)+motor.num*(motor.Iy+motor.m*motor.ry^2)+propgd.num*(propgd.Iy+propgd.m*propgd.ry^2);
TermMOI.Iz = arm.num*(arm.Iz+arm.m*arm.rz^2)+batt.num*(batt.Iz+batt.m*batt.rz^2)+centbod.num*(centbod.Iz+centbod.m*centbod.rz^2)+guid.num*(guid.Iz+guid.m*guid.rz^2)+motor.num*(motor.Iz+motor.m*motor.rz^2)+propgd.num*(propgd.Iz+propgd.m*propgd.rz^2);

TermMOI