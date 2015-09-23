function [pitch_cont] = parsePitchController(filename)

data = load(filename);

pitch_cont.time = data(:,1) - data(1,1);
pitch_cont.pitch = data(:,2);
pitch_cont.rc_p = data(:,3);
pitch_cont.rc_i = data(:,4);
pitch_cont.rc_d = data(:,5);
pitch_cont.pitch_send = data(:,6);
pitch_cont.x = data(:,7);
pitch_cont.y = data(:,8);
pitch_cont.z = data(:,9);
pitch_cont.phi = data(:,10);
pitch_cont.theta = data(:,11);
pitch_cont.psi = data(:,12);
pitch_cont.target_pitch = data(:,13);

end