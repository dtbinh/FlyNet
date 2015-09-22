clear variables
close all
clc

addpath('parse_functions');
addpath('plot_functions');

addpath('Collision_Avoidance_04_02_15');

test_num = 4;

file_ca = ['collision_avoidance', num2str(test_num),'.txt'];

ca = parseCollisionAvoidance(file_ca);

plotCollisionAvoidance(ca)

