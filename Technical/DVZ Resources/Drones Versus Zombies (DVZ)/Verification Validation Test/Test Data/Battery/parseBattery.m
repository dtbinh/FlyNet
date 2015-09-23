function [batt] = parseBattery(filename)
%time,field.header.seq,field.header.stamp,field.header.frame_id,field.ground_distance,field.flow_x,field.flow_y,field.velocity_x,field.velocity_y,field.quality

data = load(filename);

batt.time = data(:,1);
batt.voltage = data(:,2)./1000;
batt.current = data(:,3)./10./1000;
batt.batt_remaining = data(:,4);

end