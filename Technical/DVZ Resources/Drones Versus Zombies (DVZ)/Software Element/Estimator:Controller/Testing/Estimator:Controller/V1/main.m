clear all
close all
clc

global dt

dt = .01;

type = 1;
vicon_noise = .001;

Q = 3*diag([.01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01 .01]);
R = .001*diag([.1 .1 .1 .1 .1 .1]);
filter = vicon_filter(Q,R);
quad = quad([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]');

pose_list = [];
truth = [];

pose_ref = [1;1;1;0];
t = [];

for i = 1:1000
    if isempty(t)
        t = [0];
    else
        t = [t;t(i-1) + dt];
    end
    controls = controller(quad.pose,pose_ref,1);
    quad.update_pose(controls);
    vicon_measurement = quad.pose + randn(size(quad.pose))*vicon_noise;
    estimate = filter.propogate(vicon_measurement);
    estimate = filter.update(vicon_measurement);
    pose_list = [pose_list;estimate'];
    truth = [truth;quad.pose'];
end

plot(t,pose_list(:,1:3))
legend('x','y','z')

figure
plot(t,truth(:,1:3))