function [alt_cont] = parseAltController(filename)

data = load(filename);

alt_cont.time = data(:,1) - data(1,1); % system time
alt_cont.alt = data(:,2); % filtered px4flow altitude
alt_cont.rc_p = data(:,3); % altitude controller p contribution
alt_cont.rc_i = data(:,4); % altitude controller i contribution
alt_cont.rc_d = data(:,5); % altitude controller d contribution
alt_cont.t_send = data(:,6); % throttle pwm sent from altitude controller
alt_cont.x = data(:,7); % quad inertial x position from vicon
alt_cont.y = data(:,8); % quad inertial y position from vicon
alt_cont.z = data(:,9); % quad inertial z position from px4flow - unfiltered
alt_cont.phi = data(:,10); % quad roll angle from pixhawk
alt_cont.theta = data(:,11); % quad pitch angle from pixhawk
alt_cont.psi = data(:,12); % quad yaw angle from pixhawk
alt_cont.target_alt = data(:,13); % target altitude

end