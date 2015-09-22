function [sys_data] = parseSYSID(sys_file)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
data = load(sys_file);


sys_data.time = data(:,1) - data(1,1);
sys_data.target_pitch = data(:,2);
sys_data.target_roll = data(:,3);
sys_data.pitch = data(:,4);
sys_data.roll = data(:,5);


end

