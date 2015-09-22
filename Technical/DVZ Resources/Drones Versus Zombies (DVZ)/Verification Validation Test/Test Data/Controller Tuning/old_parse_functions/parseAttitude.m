function [att] = parseAttitude(filename)

data = load(filename);

att.time = data(:,1) - data(1,1); % system time
att.p_roll = data(:,2); % quad roll angle from pixhawk
att.p_pitch = data(:,3); % quad pitch angle from pixhawk
att.p_yaw = data(:,4); % quad yaw angle from pixhawk
att.v_roll = data(:,5); % quad roll angle from vicon
att.v_pitch = data(:,6); % quad pitch angle from vicon
att.v_yaw = data(:,7); % quad yaw angle from vicon

end