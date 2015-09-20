% Project DVZ
% Author: Ben Zatz
% Created 1/29/2015

function [time,accel,gyro,mag,euler,euler_rates] = loadIMU(filename)
% The purpose of this function is to load the IMU data from the Pixhawk
    data = load(filename);
    
    time = data(:,1)-data(1,1);
    accel.x = data(:,2);
    accel.y = data(:,3);
    accel.z = data(:,4);
    gyro.x = data(:,5);
    gyro.y = data(:,6);
    gyro.z = data(:,7);
    mag.x = data(:,8);
    mag.y = data(:,9);
    mag.z = data(:,10);
    euler.roll = data(:,11);
    euler.pitch = data(:,12);
    euler.yaw = data(:,13);
    euler_rates.roll = data(:,14);
    euler_rates.pitch = data(:,15);
    euler_rates.yaw = data(:,16);
end