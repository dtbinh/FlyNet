clear all
close all
clc
%%
filename = 'acc.txt';
biases = LoadBiases(filename);
PlotBiases(biases)
freq = 1/mean(diff(biases.time));

disp(['Accel Sample Rate = ',num2str(freq)])