function [log] = parseLogFile(filename)

data = csvread(filename,4,0);

%% System time
log.time = data(:,1) - data(1,1);

%% Vicon position
log.vicon_x = data(:,2);
log.vicon_y = data(:,3);
log.vicon_z = data(:,4);

%% Pixhawk attitude values
log.pixhawk_roll = data(:,5);
log.pixhawk_pitch = data(:,6);
log.pixhawk_yaw = data(:,7);
log.pixhawk_rollspeed = data(:,8);
log.pixhawk_pitchspeed = data(:,9);
log.pixhawk_yawspeed = data(:,10);

%% AMCL position and heading
log.amcl_x = data(:,11);
log.amcl_y = data(:,12);
log.amcl_z = data(:,13);
log.amcl_yaw = data(:,14);

%% Altitude controller data
log.filtered_px4flow_alt = data(:,15);
log.unfiltered_px4flow_alt = data(:,16);
log.alt_RC_P = data(:,17);
log.alt_RC_I = data(:,18);
log.alt_RC_D = data(:,19);
log.rc_T_send = data(:,20);

%% Velocity controller data
log.velx_body = data(:,21);
log.velx_P = data(:,22);
log.velx_I = data(:,23);
log.velx_D = data(:,24);
log.pitch_angle_send = data(:,25);
log.pitch_pwm_send = data(:,26);
log.vely_body = data(:,27);
log.vely_P = data(:,28);
log.vely_I = data(:,29);
log.vely_D = data(:,30);
log.roll_angle_send = data(:,31);
log.roll_pwm_send = data(:,32);
log.target_velx_inertial = data(:,33);
log.target_vely_inertial = data(:,34);
log.inertial_yaw = data(:,35);
log.target_velx_body = data(:,36);
log.target_vely_body = data(:,37);

%% Position controller data
log.velx_inertial_send = data(:,38);
log.posx_P = data(:,39);
log.posx_I = data(:,40);
log.posx_D = data(:,41);
log.vely_inertial_send = data(:,42);
log.posy_P = data(:,43);
log.posy_I = data(:,44);
log.posy_D = data(:,45);
log.target_x = data(:,46);
log.target_y = data(:,47);

%% RC handset pwm data
log.handset_roll = data(:,48);
log.handset_pitch = data(:,49);
log.handset_throttle = data(:,50);
log.handset_yaw = data(:,51);
log.handset_ch5 = data(:,52);
log.handset_ch6 = data(:,53);

%% Pixhawk battery data
log.voltage = data(:,54);
log.current = data(:,55);
log.battery_remaining = data(:,56);

%% Pixhawk raw imu data
log.pixhawk_xacc = data(:,57);
log.pixhawk_yacc = data(:,58);
log.pixhawk_zacc = data(:,59);
log.pixhawk_xgyro = data(:,60);
log.pixhawk_ygyro = data(:,61);
log.pixhawk_zgyro = data(:,62);
log.pixhawk_xmag = data(:,63);
log.pixhawk_ymag = data(:,64);
log.pixhawk_zmag = data(:,65);

%% Localization subscriber data
log.local_xmap = data(:,66);
log.local_ymap = data(:,67);
log.local_zmap = data(:,68);
log.local_velx_body = data(:,69);
log.local_vely_body = data(:,70);
log.local_velz_map = data(:,71);
log.local_accx_body = data(:,72);
log.local_accy_body = data(:,73);
log.local_accz_map = data(:,74);
log.local_psi_map = data(:,75);

%% PX4Flow subscriber data
log.px4flow_alt = data(:,76);
log.px4flow_quality = data(:,77);
log.px4flow_velx = data(:,78);
log.px4flow_vely = data(:,79);

%% Hokuyo subscriber/collision avoidance data
log.hokuyo_angle_min = data(:,80);
log.hokuyo_angle_max = data(:,81);
log.hokuyo_angle_increment = data(:,82);
log.hokuyo_range_min = data(:,83);
log.hokuyo_range_max = data(:,84);
log.hokuyo_min_dist = data(:,85);
%%%%%%%% Missing min_range %%%%%%%%%%%
log.hokuyo_angle = data(:,86);
log.hokuyo_target_velx_body = data(:,87);
log.hokuyo_target_vely_body = data(:,88);
log.hokuyo_ranges = data(:,89:end);

end