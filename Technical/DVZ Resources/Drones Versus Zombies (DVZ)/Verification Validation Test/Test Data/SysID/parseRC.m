function [rc] = parseRC(filename)
%time,field.header.seq,field.header.stamp,field.header.frame_id,field.ground_distance,field.flow_x,field.flow_y,field.velocity_x,field.velocity_y,field.quality

data = load(filename);

rc.time = data(:,1);
rc.roll = data(:,2);
rc.pitch = data(:,3);
rc.throttle = data(:,4);
rc.yaw = data(:,5);
rc.roll_send = data(:,6);
rc.pitch_send = data(:,7);
rc.throttle_send = data(:,8);
rc.yaw_send = data(:,9);


end