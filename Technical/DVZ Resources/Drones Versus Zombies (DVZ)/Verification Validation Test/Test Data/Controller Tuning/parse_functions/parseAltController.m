function [alt_cont] = parseAltController(filename)

data = load(filename);

alt_cont.time = data(:,1) - data(1,1); % system time
alt_cont.filtered_alt = data(:,2); % filtered px4flow altitude
alt_cont.rc_p = data(:,3); % altitude controller p contribution
alt_cont.rc_i = data(:,4); % altitude controller i contribution
alt_cont.rc_d = data(:,5); % altitude controller d contribution
alt_cont.t_send = data(:,6); % throttle pwm sent from altitude controller
alt_cont.vicon_x = data(:,7); % quad inertial x position from vicon
alt_cont.vicon_y = data(:,8); % quad inertial y position from vicon
alt_cont.vicon_z = data(:,9); % quad inertial z position from vicon
alt_cont.pixhawk_phi = data(:,10); % quad roll angle from pixhawk
alt_cont.pixhawk_theta = data(:,11); % quad pitch angle from pixhawk
alt_cont.pixhawk_psi = data(:,12); % quad yaw angle from pixhawk
alt_cont.target_alt = data(:,13); % target altitude
alt_cont.amcl_x = data(:,14); % quad inertial x position from amcl
alt_cont.amcl_y = data(:,15); % quad inertial y position from amcl
alt_cont.amcl_z = data(:,16); % quad inertial z position from amcl
alt_cont.amcl_psi = data(:,17); % quad inertial yaw angle from amcl
alt_cont.unfiltered_alt = data(:,18); % unfiltered px4flow altitude

end