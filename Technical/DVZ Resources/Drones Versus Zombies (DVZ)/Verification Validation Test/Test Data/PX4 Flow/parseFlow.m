function [flow] = parseFlow(filename)
%time,field.header.seq,field.header.stamp,field.header.frame_id,field.ground_distance,field.flow_x,field.flow_y,field.velocity_x,field.velocity_y,field.quality

data = load(filename);

flow.time = data(:,1);
flow.gnd_d = data(:,4);
flow.flow_x = data(:,5);
flow.flow_y = data(:,6);
flow.vel_x = data(:,7);
flow.vel_y = data(:,8);
flow.qual = data(:,9);

end