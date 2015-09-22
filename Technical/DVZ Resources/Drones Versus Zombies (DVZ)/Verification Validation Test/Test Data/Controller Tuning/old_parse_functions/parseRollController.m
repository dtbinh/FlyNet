function [roll_cont] = parseRollController(filename)

data = load(filename);

roll_cont.time = data(:,1) - data(1,1);
roll_cont.roll = data(:,2);
roll_cont.rc_p = data(:,3);
roll_cont.rc_i = data(:,4);
roll_cont.rc_d = data(:,5);
roll_cont.roll_send = data(:,6);
roll_cont.x = data(:,7);
roll_cont.y = data(:,8);
roll_cont.z = data(:,9);
roll_cont.phi = data(:,10);
roll_cont.theta = data(:,11);
roll_cont.psi = data(:,12);
roll_cont.target_roll = data(:,13);

end