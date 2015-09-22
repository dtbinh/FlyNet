function [vel_cont] = parseVelocityController(filename)

data = load(filename);

vel_cont.time = data(:,1) - data(1,1); % system time
vel_cont.velx_body = data(:,2); % quad x velocity in body frame
vel_cont.rcx_p = data(:,3); % x velocity controller p contribution
vel_cont.rcx_i = data(:,4); % x velocity controller i contribution
vel_cont.rcx_d = data(:,5); % x velocity controller d contribution
vel_cont.pitch_angle_send = data(:,6); % pitch angle sent from velocity controller
vel_cont.pitch_pwm_send = data(:,7); % pitch pwm sent from velocity controller
vel_cont.vely_body = data(:,8); % quad y velocity in body frame
vel_cont.rcy_p = data(:,9); % y velocity controller p contribution
vel_cont.rcy_i = data(:,10); % y velocity controller i contribution
vel_cont.rcy_d = data(:,11); % y velocity controller d contribution
vel_cont.roll_angle_send = data(:,12); % roll angle sent from velocity controller
vel_cont.roll_pwm_send = data(:,13); % roll pwm sent from velocity controller
vel_cont.vicon_x = data(:,14); % quad inertial x position from vicon
vel_cont.vicon_y = data(:,15); % quad inertial y position from vicon
vel_cont.vicon_z = data(:,16); % quad inertial z position from vicon
vel_cont.pixhawk_phi = data(:,17); % quad roll angle from pixhawk
vel_cont.pixhawk_theta = data(:,18); % quad pitch angle from pixhawk
vel_cont.pixhawk_psi = data(:,19); % quad yaw angle from pixhawk
vel_cont.target_velx_inertial = data(:,20); % target x velocity in inertial frame
vel_cont.target_vely_inertial = data(:,21); % target y velocity in inertial frame
vel_cont.target_velx_body = data(:,22); % target x velocity in body frame
vel_cont.target_vely_body = data(:,23); % target y velocity in body frame
vel_cont.amcl_x = data(:,24); % quad inertial x position from amcl
vel_cont.amcl_y = data(:,25); % quad inertial y position from amcl
vel_cont.amcl_z = data(:,26); % quad inertial z position from amcl
vel_cont.amcl_psi = data(:,27); % quad inertial yaw angle from amcl

end