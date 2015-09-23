function [att] = parseAttitude(filename)
%time,field.header.seq,field.header.stamp,field.header.frame_id,field.ground_distance,field.flow_x,field.flow_y,field.velocity_x,field.velocity_y,field.quality

data = load(filename);

% att.time = data(:,1);
% att.p_roll = data(:,2);
% att.p_pitch = data(:,3);
% att.p_yaw = data(:,4);
% att.v_roll = wrapToPi(data(:,5));
% att.v_pitch = wrapToPi(-1*data(:,6));
% att.v_yaw = wrapToPi(-1*data(:,7) + 2.709447547215497);

att.time = data(:,1);
att.p_roll = data(:,2);
att.p_pitch = data(:,3);
att.p_yaw = data(:,4);
att.v_roll = wrapToPi(-1*data(:,6));
att.v_pitch = wrapToPi(data(:,5));
att.v_yaw = wrapToPi(-1*data(:,7) + 2.709447547215497);

end