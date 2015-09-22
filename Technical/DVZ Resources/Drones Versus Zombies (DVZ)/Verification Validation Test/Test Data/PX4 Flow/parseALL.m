function [flow,vicon] = parseALL(filename)
%time,field.header.seq,field.header.stamp,field.header.frame_id,field.ground_distance,field.flow_x,field.flow_y,field.velocity_x,field.velocity_y,field.quality

data = load(filename);

flow.time = data(:,1);
vicon.x = data(:,2);
vicon.y = data(:,3);
vicon.z = data(:,4);
flow.vel_x = data(:,5);
flow.vel_y = data(:,6);
flow.gnd_d = data(:,7);
flow.qual = data(:,8);
end