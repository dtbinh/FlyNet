clear variables
close all
clc

pose = [0, 1, 2;...
        0, 0, 0;...
        0, 0, pi/4];

figure
hold on
grid on
axis equal
for i = 1:3
    plot_heading(pose(:,i),1);
    waitforbuttonpress()
end