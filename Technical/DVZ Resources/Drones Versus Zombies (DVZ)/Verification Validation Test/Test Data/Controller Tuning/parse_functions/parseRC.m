function [rc] = parseRC(filename)

data = load(filename);

rc.time = data(:,1) - data(1,1);
rc.roll = data(:,2);
rc.pitch = data(:,3);
rc.throttle = data(:,4);
rc.yaw = data(:,5);
rc.roll_send = data(:,6);
rc.pitch_send = data(:,7);
rc.throttle_send = data(:,8);
rc.yaw_send = data(:,9);
rc.x = data(:,10);
rc.y = data(:,11);
rc.z = data(:,12);
rc.phi = data(:,13);
rc.theta = data(:,14);
rc.psi = data(:,15);

end