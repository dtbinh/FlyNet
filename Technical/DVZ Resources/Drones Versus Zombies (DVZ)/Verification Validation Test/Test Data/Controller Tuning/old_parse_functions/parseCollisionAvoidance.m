function [ca] = parseCollisionAvoidance(filename)

data = csvread(filename,4,0);

ca.time = data(:,1) - data(1,1); % system time
ca.angle_min = data(:,2); % minimum scan angle
ca.angle_max = data(:,3); % maximum scan angle
ca.angle_inc = data(:,4); % scan angle increment
ca.range_min = data(:,5); % scan range minimum
ca.range_max = data(:,6); % scan range maximum
ca.min_dist = data(:,7); % minimum distance
ca.min_range = data(:,8); % minimum range from scan
ca.angle = data(:,9); % angle at which minimum range occurs
ca.target_velx_body = data(:,10); % commanded x velocity in body frame
ca.target_vely_body = data(:,11); % commanded y velocity in body frame
ca.ranges = data(:,12:end); % laser scan ranges

end