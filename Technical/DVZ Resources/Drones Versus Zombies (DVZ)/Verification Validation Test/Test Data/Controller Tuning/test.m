clear variables
close all
clc

syms psi

R = [cos(psi), sin(psi);...
     sin(psi), -cos(psi)];
Rinv = inv(R)