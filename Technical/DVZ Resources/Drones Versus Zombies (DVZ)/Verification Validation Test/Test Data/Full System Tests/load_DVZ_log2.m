function log = load_DVZ_log(filename)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load_DVZ_log.m
% Austin Lillard
% Created: 04/06/2015
% Updated: 04/06/2015
% Purpose:
%       -To load the specified log file generated from a flight test for
%       data analysis.
%
% Input:
%       -filename: name of the data file
% 
% Output:
%       -log: structure containing logged information
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load in data

% Load to structure
data = csvread(filename,4,0);

% Assign to output data structure
%% System time
log.time = data(:,1) - data(1,1);      % [ns?]

%% Vicon position
log.vicon_x = data(:,2);   % [m]
log.vicon_y = data(:,3);    % [m]
log.vicon_z = data(:,4);    % [m]
log.vicon_roll = data(:,5);  % [rad]
log.vicon_pitch = data(:,6);
log.vicon_yaw = data(:,7);	% [rad]

%% Pixhawk attitude values
log.pixhawk_roll = data(:,8); % [rad]
log.pixhawk_pitch = data(:,9); % [rad]
log.pixhawk_yaw = data(:,10); % [rad]
log.pixhawk_rollspeed = data(:,11); % [rad/s]
log.pixhawk_pitchspeed  = data(:,12); % [rad/s]
log.pixhawk_yawspeed = data(:,13); % [rad/s]

%% AMCL position and heading
log.amcl_x = data(:,14);    % [m]
log.amcl_y = data(:,15);    % [m]
log.amcl_z = data(:,16);    % [m]
log.amcl_yaw = data(:,17);  % [rad]

%% Altitude controller data
log.filtered_px4flow_alt = data(:,18); %[m]
log.unfiltered_px4flow_alt = data(:,19); %[m]
log.alt_RC_P = data(:,20);  % [PWM]
log.alt_RC_I = data(:,21);  % [PWM]
log.alt_RC_D = data(:,22);  % [PWM]
log.rc_T_send = data(:,23); % [PWM]

%% Velocity controller data
log.velx_body = data(:,24); % [m/s]
log.velx_P = data(:,25);    % [PWM]
log.velx_I = data(:,26);    % [PWM]
log.velx_D = data(:,27);    % [PWM]
log.pitch_angle_send = data(:,28); % [rad]
log.pitch_pwm_send = data(:,29); % [PWM]
log.vely_body = data(:,30); % [m/s]
log.vely_P = data(:,31);    % [PWM]
log.vely_I = data(:,32);    % [PWM]
log.vely_D = data(:,33);    % [PWM]
log.roll_angle_send = data(:,34);   % [rad]
log.roll_pwm_send = data(:,35); % [PWM]
log.target_velx_inertial = data(:,36);  % [m/s]
log.target_vely_inertial = data(:,37);  % [m/s]
log.target_velx_body = data(:,38);  % [m/s]
log.target_vely_body = data(:,39);  % [m/s]

%% Yaw controller



%% Position controller data
log.velx_inertial_send = data(:,40);    % [m]
log.posx_P = data(:,41);    % [PWM]
log.posx_I = data(:,42);    % [PWM]
log.posx_D = data(:,43);    % [PWM]
log.vely_inertial_send = data(:,44);    % [m]
log.posy_P = data(:,45);    % [PWM]
log.posy_I = data(:,46);    % [PWM]
log.posy_D = data(:,47);    % [PWM]
log.target_x = data(:,48);  % [m]
log.target_y = data(:,49);  % [m]

%% RC handset pwm data
log.handset_roll = data(:,50);  % [PWM]
log.handset_pitch = data(:,51); % [PWM]
log.handset_throttle = data(:,52);  % [PWM]
log.handset_yaw = data(:,53);   % [PWM]
log.handset_ch5 = data(:,54);   % [PWM]
log.handset_ch6 = data(:,55);   % [PWM]

%% Pixhawk battery data
log.voltage = data(:,56);   % [volts]
log.current = data(:,57);   % [amps]
log.battery_remaining = data(:,58); % [percentage]

%% Pixhawk raw imu data
log.pixhawk_xacc = data(:,59);
log.pixhawk_yacc = data(:,60);
log.pixhawk_zacc = data(:,61);
log.pixhawk_xgyro = data(:,62);
log.pixhawk_ygyro = data(:,63);
log.pixhawk_zgyro = data(:,64);
log.pixhawk_xmag = data(:,65);
log.pixhawk_ymag = data(:,66);
log.pixhawk_zmag = data(:,67);

%% Localization subscriber data
log.local_xmap = data(:,68);
log.local_ymap = data(:,69);
log.local_zmap = data(:,70);
log.local_velx_body = data(:,71);
log.local_vely_body = data(:,72);
log.local_velz_map = data(:,73);
log.local_accx_body = data(:,74);
log.local_accy_body = data(:,75);
log.local_accz_map = data(:,76);
log.local_psi_map = data(:,77);

%% PX4Flow subscriber data
log.px4flow_alt = data(:,78);
log.px4flow_quality = data(:,79);
log.px4flow_velx = data(:,80);
log.px4flow_vely = data(:,81);

%% Kalman filter data
log.kalman_xdot_body = data(:,82);
log.kalman_ydot_body = data(:,83);
log.kalman_zdot_body = data(:,84);
log.kalman_ax_bias_body = data(:,85);
log.kalman_ay_bias_body = data(:,86);
log.kalman_az_bias_body = data(:,87);
log.kalman_phi_bias = data(:,88);
log.kalman_theta_bias = data(:,89);

log.kalman_xdot_body_var = data(:,90);
log.kalman_ydot_body_var = data(:,91);
log.kalman_zdot_body_var = data(:,92);
log.kalman_ax_bias_body_var = data(:,93);
log.kalman_ay_bias_body_var = data(:,94);
log.kalman_az_bias_body_var = data(:,95);
log.kalman_phi_bias_var = data(:,96);
log.kalman_theta_bias_var = data(:,97);

%% Hokuyo subscriber/collision avoidance data
log.hokuyo_angle_min = data(:,98);
log.hokuyo_angle_max = data(:,99);
log.hokuyo_angle_increment = data(:,100);
log.hokuyo_range_min = data(:,101);
log.hokuyo_range_max = data(:,102);
log.hokuyo_min_dist = data(:,103);
log.hokuyo_min_range = data(:,104);
log.hokuyo_angle = data(:,105);
log.hokuyo_target_velx_body = data(:,106);
log.hokuyo_target_vely_body = data(:,107);
log.hokuyo_ranges = data(:,108:end);

end
