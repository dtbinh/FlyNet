function [alt] = parseAltController(filename)
%time,field.header.seq,field.header.stamp,field.header.frame_id,field.ground_distance,field.flow_x,field.flow_y,field.velocity_x,field.velocity_y,field.quality

data = load(filename);

alt.time = data(:,1);
alt.alt = data(:,2);
alt.rc_p = data(:,3);
alt.rc_i = data(:,4);
alt.rc_d = data(:,5);
alt.t_send = data(:,6);
alt.x = data(:,7);
alt.y = data(:,8);
alt.z = data(:,9);
alt.phi = data(:,10);
alt.theta = data(:,11);
alt.psi = data(:,12);

end