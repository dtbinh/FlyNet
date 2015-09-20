clear all
clc
close all

% Set up time vectors
dt = .1;
t_vec = 0:dt:20;

% Standard deviation on particle odometry, laser measurements, and flow
% sensor measurements
sigma_odom = .5;
sigma_laser = .05;
sigma_vel = .2;

% Establish ground truth position
pose_vec = [5*sin(.5*t_vec'),5*cos(.5*t_vec')];
% Establish ground truth velocity
vel_vec = [2.5*cos(.5*t_vec'),-2.5*sin(.5*t_vec')];
% Establish ground truth laser measurements to walls positioned along x =
% 10, x = -10, y = 10, and y = -10
laser_meas_truth = [abs(10*ones(size(pose_vec,1),1) - pose_vec(:,1)), ...
    abs(-10*ones(size(pose_vec,1),1) - pose_vec(:,1)),...
    abs(10*ones(size(pose_vec,1),1) - pose_vec(:,2)),...
    abs(-10*ones(size(pose_vec,1),1) - pose_vec(:,2))];
% Generate noisy measurements by adding appropriate gaussian noise to laser
% and velocity measurements
laser_meas_noise = laser_meas_truth + randn(size(laser_meas_truth))*sigma_laser;
flow_meas_noise = vel_vec + randn(size(vel_vec))*sigma_vel;

% Initialize pose estimate and list for storing pose estimates
pose_est = pose_vec(1,:);
pose_est_list = [];

% Number of particles to use
n = 500;
% Generate intial particles
%particles = randn(n,2)*sigma_odom + [pose_est(1)*ones(n,1),...
%    pose_est(2)*ones(n,1)];
particles = randn(n,2)*10;
%figure
for i = 1:length(t_vec)
%     clf
%     scatter(particles(:,1),particles(:,2),'bx'),hold on
%     scatter(pose_vec(i,1),pose_vec(i,2),50,'go')
%     axis([-10 10 -10 10])
%     pause(.1)
    % Calculate probability of each particle based on the 4D  measurement gaussian
    particle_meas = [abs(10*ones(size(particles,1),1) - particles(:,1)), ...
    abs(-10*ones(size(particles,1),1) - particles(:,1)),...
    abs(10*ones(size(particles,1),1) - particles(:,2)),...
    abs(-10*ones(size(particles,1),1) - particles(:,2))];
    
    p = mvnpdf(particle_meas,laser_meas_noise(i,:),[sigma_laser^2 0 0 0;0 sigma_laser^2 0 0;...
        0 0 sigma_laser^2 0;0 0 0 sigma_laser^2]);
    % Normalize the probabilities
    p = p./sum(p);
    % Get the cumulative sum for sampling
    cs = cumsum(p);
    % Reinitialize new particle list
    particles_new = [];
    % Resample from particles
    for j = 1:n
        r = rand;
        for k = 1:n-1
            if cs(k) > r
                particles_new = [particles_new;particles(k,:)];
                break
            end
        end
    end
    % Set particles to new list
    particles = particles_new;
    % Update the plot of particles/truth
%     clf
%     scatter(particles(:,1),particles(:,2),'bx'),hold on
%     scatter(pose_vec(i,1),pose_vec(i,2),50,'go')
%     axis([-10 10 -10 10])
%     pause(.1)
    % Re-estimate the mean
    pose_est = mean(particles);
    % Save in list for plotting
    pose_est_list = [pose_est_list;pose_est];
    % Update pose_est based on flow measurement
    pose_est = pose_est + flow_meas_noise(i,:)*dt;
    % Update particles based on flow measurement and odom noise
    particles = particles + [flow_meas_noise(i,1)*ones(size(particles,1),1),...
        flow_meas_noise(i,2)*ones(size(particles,1),1)]*dt + ...
        randn(size(particles))*sigma_odom;
    
end
% Plot estimates
close all

figure
subplot(2,2,1)
plot(t_vec,[pose_vec(:,1),pose_est_list(:,1)])
xlabel('time (s)')
ylabel('x pose (m)')
legend('truth','estimate')
subplot(2,2,2)
plot(t_vec,[pose_vec(:,2),pose_est_list(:,2)])
xlabel('time (s)')
ylabel('y pose (m)')
legend('truth','estimate')
subplot(2,2,3)
plot(t_vec,pose_est_list(:,1) - pose_vec(:,1))
xlabel('time (s)')
ylabel('error (m)')
subplot(2,2,4)
plot(t_vec,pose_est_list(:,2) - pose_vec(:,2))
xlabel('time (s)')
ylabel('error (m)')