% PX4Flow post processing
% Project DVZ
% Author: Ben Zatz
% Created 1/16/2015
% Updated 1/22/2015

clear all
close all
clc

format long

addpath(genpath('2-12-2015'));


%% Load data
filename = 'px4flow_test12.txt';
[flow,vicon] = parseALL(filename);
flow.time = flow.time - flow.time(1);

% Calculate x,y position from x,y velocity
flow.pos_x = integrateVelocity(flow.time,flow.vel_x,0);
flow.pos_y = integrateVelocity(flow.time,flow.vel_y,0);

%% Stats
% Velocity_X_mean = mean(flow.vel_x(flow.time < 60));
% Velocity_Y_mean = mean(flow.vel_y(flow.time < 60));
% Z_mean = mean(flow.gnd_d(flow.time < 60));
% 
% Velocity_X_var = (std(flow.vel_x(flow.time < 60)))^2;
% Velocity_Y_var = (std(flow.vel_y(flow.time < 60)))^2;
% Z_var = (std(flow.gnd_d(flow.time < 60)))^2;
%% Plot flow ground distance
% figure
% plot(flow.time,flow.gnd_d,'r')
% title('PX4Flow Time versus Altitude')
% xlabel('Time [s]')
% ylabel('Ground Distance [m]')

%% Plot flow x,y velocity
figure
title('PX4Flow Time versus X and Y Velocity')
subplot(311)
plot(flow.time,flow.vel_x,'b'),grid
xlabel('Time [s]')
ylabel('Velocity X [m/s]')

subplot(312)
plot(flow.time,flow.vel_y,'b'),grid
xlabel('Time [s]')
ylabel('Velocity Y [m/s]')

subplot(313)
plot(flow.time,flow.qual),grid
xlabel('Time [s]');
ylabel('Quality Factor');
title('Time versus Quality Factor','fontsize',10,'fontweight','bold');

%% Plot flow x,y integrated position
figure
title('PX4Flow Time versus X and Y Position')
subplot(211)
plot(flow.time(1:end-1),flow.pos_x(1:end-1),'b.'),grid
xlabel('Time [s]')
ylabel('Position X [m]')

subplot(212)
plot(flow.time(1:end-1),flow.pos_y(1:end-1),'b.'),grid
xlabel('Time [s]')
ylabel('Position Y [m]')

%% Plot vicon x,y,z position and flow x,y,z
figure
plot3(vicon.x(1:end-1),vicon.y(1:end-1),vicon.z(1:end-1),'b.')
hold on
grid on
plot3(flow.pos_x(1:end-1),flow.pos_y(1:end-1),flow.gnd_d(1:end-1),'r.')
title('3D Position over Time')
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
legend('VICON','PX4Flow','Location','best');

%% Plot flow ground distance and vicon z position
figure
hold on
grid on
plot(flow.time,vicon.z,'b.');
plot(flow.time,flow.gnd_d,'r.');
title('Vicon versus PX4Flow altitude')
ylabel('Ground Distance [m]')
legend('Vicon','PX4Flow','Location','Best')

%% Plot quality
figure
grid on
plot(flow.time,flow.qual)
xlabel('Time [s]','fontsize',12);
ylabel('Quality Factor','fontsize',12);
title('Time versus Quality Factor','fontsize',12,'fontweight','bold');