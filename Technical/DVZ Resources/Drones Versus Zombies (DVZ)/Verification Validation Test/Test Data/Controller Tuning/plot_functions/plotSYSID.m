function [] = plotSYSID(sys_data)
% %UNTITLED Summary of this function goes here
% %   Detailed explanation goes here
% data = load(sys_file);
% 
% 
% sys_data.time = data(1,:);
% sys_data.target_pitch = data(2,:);
% sys_data.target_roll = data(3,:);
% sys_data.pitch = data(4,:);
% sys_data.roll = data(5,:);
% 
% 

rad2deg = 180/pi;

figure
hold on
plot(sys_data.time, sys_data.target_pitch,'r')
plot(sys_data.time, rad2deg*sys_data.pitch)
title('Pitch v. Time')
ylabel('\theta (deg)')
xlabel('Time (s)')
legend('Target Pitch','Actual Pitch')

figure
hold on
plot(sys_data.time, sys_data.target_roll,'r')
plot(sys_data.time, rad2deg*sys_data.roll)
title('Roll v. Time')
ylabel('\phi (deg)')
xlabel('Time (s)')
legend('Target Roll','Actual Roll')
end