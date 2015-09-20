function [flow] = parseFlow(filename)

data = load(filename);

flow.time = data(:,1) - data(1,1); % system time
flow.target_alt = data(:,2); % target altitude
flow.flow_alt = data(:,3); % altitude from px4flow
flow.quality = data(:,4); % quality from px4flow
flow.x = data(:,5); % quad inertial x position from vicon
flow.y = data(:,6); % quad inertial y position from vicon
flow.z = data(:,7); % quad inertial z position from vicon
flow.phi = data(:,8); % quad roll angle from pixhawk
flow.theta = data(:,9); % quad pitch angle from pixhawk
flow.psi = data(:,10); % quad yaw angle from pixhawk

end