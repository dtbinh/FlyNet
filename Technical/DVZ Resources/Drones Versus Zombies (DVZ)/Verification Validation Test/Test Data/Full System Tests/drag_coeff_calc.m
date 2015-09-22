clear all
close all
clc

addpath('04_21_2015')
log = load_DVZ_log('full_system_test20.txt');

dt = diff(log.time);
M = 3;
g = 9.81;

vel_xnoise = diff(log.vicon_x)./dt;
vel_y = diff(log.vicon_y)./dt;
vel_z = diff(log.vicon_z)./dt;

ratio = .15;
for i = 1:length(vel_xnoise)
    if i == 1
        vel_x(i) = vel_xnoise(i);
    else
        vel_x(i) = ratio*vel_xnoise(i) + (1-ratio)*vel_xnoise(i-1);
    end
end

R = []

ax_noise = diff(vel_x')./dt(2:end);

ratio = .15;
for i = 1:length(ax_noise)
    if i == 1
        ax(i) = ax_noise(i);
    else
        ax(i) = ratio*ax_noise(i) + (1-ratio)*ax_noise(i-1);
    end
end
ax = ax';
%ax = ax_noise;
ay = diff(vel_y)./dt(2:end);
az = diff(vel_z)./dt(2:end);

vel_x = vel_x(2:end);
ax = ax(1:end);
time = log.time(3:end);
% mu/M * v = a_drag
% T*(cos(phi)*cos(theta) - mg = Fz
% T = m*(az + g)/(cos(phi)*cos(theta))
% ax = -g*sin(theta) - mu/m * u
% ay = T*cos(phi)/m - ay_drag
theta = log.pixhawk_pitch;
phi = log.pixhawk_roll;

% right_side = -(ax + g*sin(theta(3:end)));
vel_x = vel_x';
data_old = [time,vel_x,ax];
data = [];

for i = 1:length(data_old)
    if abs(data_old(i,3)) < 1
        data = [data;data_old(i,:)];
    end
end

p = polyfit(data(:,2),data(:,3),1);
mu = p(1)*M;

plot(data(:,1),data(:,3)),hold on
plot(data(:,1),-mu/M*data(:,2))

mu