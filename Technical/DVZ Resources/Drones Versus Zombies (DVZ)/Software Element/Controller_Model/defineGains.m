function [] = defineGains
%%%%% Current Gains: 3/9/15 %%%%%
% Roll Rate Gains: Pr_phi = 0.2*100; Ir_phi = -0.005*100; Dr_phi = 0.12*100;
% Pitch Rate Gains: Pr_theta = 0.2*100; Ir_theta = -0.005*100; Dr_theta = 0.12*100;
% Roll Angle Gains: Pa_phi = 10;
% Pitch Angle Gains: Pa_theta = 10;
% Velocity X Gains: Pvx = -0.26; Ivx = -0.05; Dvx = 0;
% Velocity Y Gains: Pvy = 0.14; Ivy = 0.05; Dvy = 0;
% Position X Gains: Px = 0.5; Ix = 0.02; Dx = 0;
% Position Y Gains: Py = 0.4; Iy = 0.03; Dy = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Roll Rate Gains
Pr_phi = 0.2*100;
Ir_phi = -0.005*100;
Dr_phi = 0.12*100;

% Pitch Rate Gains
Pr_theta = 0.2*100;
Ir_theta = -0.005*100;
Dr_theta = 0.12*100;

% Yaw Rate Gains
Pr_psi = 0.2*100;
Ir_psi = -0.005*100;
Dr_psi = 0.12*100;

% Roll Angle Gains
Pa_phi = 10;

% Pitch Angle Gains
Pa_theta = 10;

% Yaw Angle Gains
Pa_psi = 10;

% Velocity X Gains
Pvx = -0.26;
Ivx = -0.05;
Dvx = 0;

% Velocity Y Gains
Pvy = 0.14;
Ivy = 0.05;
Dvy = 0;

% Position X Gains
Px = 0.7;
Ix = 0;
Dx = 0;

% Position Y Gains
Py = 0.7;
Iy = 0;
Dy = 0;

gains.Pr_phi = Pr_phi;
gains.Ir_phi = Ir_phi;
gains.Dr_phi = Dr_phi;
gains.Pr_theta = Pr_theta;
gains.Ir_theta = Ir_theta;
gains.Dr_theta = Dr_theta;
gains.Pr_psi = Pr_psi;
gains.Ir_psi = Ir_psi;
gains.Dr_psi = Dr_psi;
gains.Pa_phi = Pa_phi;
gains.Pa_theta = Pa_theta;
gains.Pa_psi = Pa_psi;
gains.Pvx = Pvx;
gains.Ivx = Ivx;
gains.Dvx = Dvx;
gains.Pvy = Pvy;
gains.Ivy = Ivy;
gains.Dvy = Dvy;
gains.Px = Px;
gains.Ix = Ix;
gains.Dx = Dx;
gains.Py = Py;
gains.Iy = Iy;
gains.Dy = Dy;

writeGains(gains,'current_gains.txt');

end