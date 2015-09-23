function [yaw_cont] = parseYawController(filename)

data = load(filename);

yaw_cont.time = data(:,1) - data(1,1);
yaw_cont.yaw = data(:,2);
yaw_cont.rc_p = data(:,3);
yaw_cont.rc_i = data(:,4);
yaw_cont.rc_d = data(:,5);
yaw_cont.yaw_send = data(:,6);
yaw_cont.x = data(:,7);
yaw_cont.y = data(:,8);
yaw_cont.z = data(:,9);
yaw_cont.phi = data(:,10);
yaw_cont.theta = data(:,11);
yaw_cont.psi = data(:,12);
yaw_cont.target_yaw = data(:,13);

end