function [gains] = readGains(filename)

data = load(filename);

gains.Pr_phi = data(1,1);
gains.Ir_phi = data(1,2);
gains.Dr_phi = data(1,3);
gains.Pr_theta = data(1,4);
gains.Ir_theta = data(1,5);
gains.Dr_theta = data(1,6);
gains.Pr_psi = data(1,7);
gains.Ir_psi = data(1,8);
gains.Dr_psi = data(1,9);
gains.Pa_phi = data(1,10);
gains.Pa_theta = data(1,11);
gains.Pa_psi = data(1,12);
gains.Pvx = data(1,13);
gains.Ivx = data(1,14);
gains.Dvx = data(1,15);
gains.Pvy = data(1,16);
gains.Ivy = data(1,17);
gains.Dvy = data(1,18);
gains.Px = data(1,19);
gains.Ix = data(1,20);
gains.Dx = data(1,21);
gains.Py = data(1,22);
gains.Iy = data(1,23);
gains.Dy = data(1,24);

end
