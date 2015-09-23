% IMU post processing
% Project DVZ
% Author: Ben Zatz
% Created 1/29/2015

clear all
close all
clc

format long

addpath('1-29-2015')

%% Load data
filename = 'imu_test2.txt';
[time,accel,gyro,mag,euler,euler_rates] = loadIMU(filename);

%% Plotting Accelerometer data
figure(1)
subplot(311)
plot(time,accel.x),grid;
title('Accelerometer versus Time')
ylabel('X [mg]');

subplot(312)
plot(time,accel.y),grid;
ylabel('Y [mg]');

subplot(313)
plot(time,accel.z),grid;
ylabel('Z [mg]');
xlabel('Time [s]');

x_doubledot_mean = mean(accel.x)
y_doubledot_mean = mean(accel.y)
z_doubledot_mean = mean(accel.z)

x_doubledot_StandDev = (std(accel.x))^2
y_doubledot_StandDev = (std(accel.y))^2
z_doubledot_StandDev = (std(accel.z))^2
%% Plotting Gyro data
figure(2)
subplot(311)
plot(time,gyro.x),grid;
title('Gyro versus Time')
ylabel('X [millirad/sec]');

subplot(312)
plot(time,gyro.y),grid;
ylabel('Y [millirad/sec]');

subplot(313)
plot(time,gyro.z),grid;
ylabel('Z [millirad/sec]');
xlabel('Time [s]');

%% Plotting Magnetometer data
figure(3)
subplot(311)
plot(time,mag.x),grid;
title('Magnetometer versus Time')
ylabel('X [milli tesla]');

subplot(312)
plot(time,mag.y),grid;
ylabel('Y [milli tesla]');

subplot(313)
plot(time,mag.z),grid;
ylabel('Z [milli tesla]');
xlabel('Time [s]');

%% Plotting Euler Angle data
figure(4)
subplot(311)
plot(time,euler.roll),grid;
title('Euler Angles versus Time')
ylabel('Roll [rad (-\pi:+\pi)]');

subplot(312)
plot(time,euler.pitch),grid;
ylabel('Pitch [rad (-\pi:+\pi)]');

subplot(313)
plot(time,euler.yaw),grid;
ylabel('Yaw [rad (-\pi:+\pi)]');
xlabel('Time [s]');

%% Plotting Euler Angle rates data
figure(5)
subplot(311)
plot(time,euler_rates.roll),grid;
title('Euler Angle Rates versus Time')
ylabel('Roll [rad/sec]');

subplot(312)
plot(time,euler_rates.pitch),grid;
ylabel('Pitch [rad/sec]');

subplot(313)
plot(time,euler_rates.yaw),grid;
ylabel('Yaw [rad/sec]');
xlabel('Time [s]');