function [t,pose_data,pose_truth,vel_truth] = generate_pose_data(type,pose_noise)
global vicon_dt

dt = vicon_dt;
t = 0:dt:10;

if type == 1
    pose_truth = [5*sin(.1*t);5*cos(.1*t);1.25*ones(size(t));...
        .1*sin(.1*t);.1*cos(.1*t);2*pi*sin(.1*t)];
    pose_data = pose_truth + pose_noise*randn(size(pose_truth));
end

t = t';
pose_truth = pose_truth';
vel_truth = diff(pose_truth)/dt;
pose_data = pose_data';
end