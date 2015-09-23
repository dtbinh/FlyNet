function [pos_cont] = parsePositionController(filename)

data = load(filename);

pos_cont.time = data(:,1) - data(1,1); % system time
pos_cont.velx_inertial_send = data(:,2); % quad x velocity in inertial frame sent from position controller
pos_cont.rcx_p = data(:,3); % x position controller p contribution
pos_cont.rcx_i = data(:,4); % x position controller i contribution
pos_cont.rcx_d = data(:,5); % x position controller d contribution
pos_cont.vely_inertial_send = data(:,6); % quad y velocity in inertial frame sent from position controller
pos_cont.rcy_p = data(:,7); % y position controller p contribution
pos_cont.rcy_i = data(:,8); % y position controller i contribution
pos_cont.rcy_d = data(:,9); % y position controller d contribution
pos_cont.x = data(:,10); % quad inertial x position from vicon
pos_cont.y = data(:,11); % quad inertial y position from vicon
pos_cont.z = data(:,12); % quad inertial z position from px4flow
pos_cont.phi = data(:,13); % quad roll angle from pixhawk
pos_cont.theta = data(:,14); % quad pitch angle from pixhawk
pos_cont.psi = data(:,15); % quad yaw angle from pixhawk
pos_cont.target_x = data(:,16); % target x position in inertial frame
pos_cont.target_y = data(:,17); % target y position in inertial frame

end