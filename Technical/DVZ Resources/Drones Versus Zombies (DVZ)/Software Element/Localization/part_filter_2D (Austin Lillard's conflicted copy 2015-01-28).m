
%% Clean up
clear all
close all
clc

profile on
%% Initialize

animate = 1;            % turn animation on and off

sigma_laser = 0.1;       %std dev of range measurement [m]

sigma_odom = 0.5;       % std dev of odometry data [m]

num_particles = 50;     % number of particles being tracked

num_beams = 15;

t = linspace(0, 20, 20)'; % time [s]

x0 = 17;                 % x initial true position [m]
y0 = 13;                 % y initial true position [m]
psi0 = 0;                % psi initial true position [rad]

%% Load Map data

% Read map
map = imread('map_example.png');

% size of map
[m, n] = size(map);

% Convert to grayscale, get rid of midtones
map = mat2gray(map, [80 100]);

% Invert definition of black and white (1==black, 0==white)
map = imcomplement(map);

% Make all gray areas black
map(map > 0) = 1;

% Resolution of the map, length of unit square [m]
map_res = 0.05;

% maximum x and y values allowable on the map
map_max_x = map_res*n;
map_max_y = map_res*m;



%% True position

% movement pattern of object [m]
movement_x = 2*sin(0.5*t);    
% movement_y = zeros(length(t), 1);
movement_y = 0.1*sin(0.5*t);
movement_psi = zeros(length(t), 1);

% true position moves with defined movement pattern
x_true = x0*ones(length(t), 1) + movement_x; 
y_true = y0*ones(length(t), 1) + movement_y;
psi_true = psi0*ones(length(t), 1) + movement_psi;

%% Particle Array Creation
% Create array, [x_part, y_part], num_particles x 2
part_array = zeros(num_particles, 2);

gauss_widthx = 2;
gauss_widthy = 2;
% Initialize particles
% part_array(:, 1) = map_max_x*rand(num_particles, 1);
% part_array(:, 2) = map_max_y*rand(num_particles, 1);
part_array(:,1) = x_true(1) - gauss_widthx + 2*gauss_widthx*rand(num_particles, 1);
part_array(:,2) = y_true(1) - gauss_widthy + 2*gauss_widthy*rand(num_particles, 1);

% Allocate Arrays
part_bin = zeros(num_particles, 1);
part_new = zeros(num_particles, 2);
part_med = zeros(length(t), 2);
part_mean = zeros(length(t), 2);
part_meas = zeros(num_particles, num_beams);
true_meas = zeros(length(t), num_beams);
part_prob = zeros(num_particles, 1);

for n = 2:length(t)
    
    % Update Particles from odometry info, x and y positions.
    % Record particle positions at each time step for plotting
    part_array(:, 1, n) = part_array(:, 1, n-1) + movement_x(n);
    part_array(:, 2, n) = part_array(:, 2, n-1) + movement_y(n);
    
    % Add odometry noise
    part_array(:, :, n) = part_array(:, :,  n) + sigma_odom*randn(num_particles, 2);
    
    % Expected Measurements of each particle [m]
    for ii = 1:num_particles
        
        % x, y, and psi values of particle.  For ease of entering into
        % function.
       x_part = part_array(ii, 1, n);
       y_part = part_array(ii, 2, n);
       psi_part = psi_true(n);
        
       % Laser scan ranges returned by particle at that location with given
       % psi value.
        part_meas(ii, :) = laser_scan(x_part, y_part, psi_part, num_beams, ...
            map, map_res);
        
    end
    
    % Actual Measurement
    true_meas = laser_scan(x_true(n), y_true(n), psi_true(n), num_beams, ...
        map, map_res);
    
    % Calculate probabilities of each partcle by summing the beam
    % probabilites.
    for ii = 1:num_particles
        for jj = 1:num_beams
             part_prob(ii) = part_prob(ii)+ normpdf(part_meas(ii, jj), true_meas(jj), sigma_laser);
        end
    end
    
    % Normalize to 1
    part_prob = part_prob./sum(part_prob);
    
    part_bin(1) = part_prob(1);
    % Create distribution bins, 0 to 1
    for ii = 2:num_particles
       
        part_bin(ii) = part_prob(ii) + part_bin(ii-1);
        
    end
    
    % Randomly sample from 0 to 1
    bin_sample = rand(num_particles, 1);
    
    % Which particle positions to keep
    for ii = 1:length(bin_sample)
        
       % difference betweened sampled value and bin value for each bin 
       bin_diff = bin_sample(ii) - part_bin;
       
       % calculate which bin the sample landed in
       bin_num = length(bin_diff(bin_diff >= 0)) + 1;
       
       % New particle locations
       part_new(ii, :) = part_array(bin_num, :, n); 
       
    end
    
    part_array(:, :, n) = part_new;
    
    % Median particle value
    part_med(n,1) = median(part_array(:, 1, n));
    part_med(n,2) = median(part_array(:, 2, n));
    part_mean(n,1) = mean(part_array(:, 1, n));
    part_mean(n,2) = mean(part_array(:, 2, n));
end

%% Plot results
% Estimated position
figure
subplot(2,1,1)
plot(t, part_med(:,1), 'r')
hold on
plot(t, part_mean(:,1), 'b')
plot(t, x_true, '--g');
ylabel('X [m]');
hold off
legend('Median', 'Mean', 'True');
title('Position Estimation')

subplot(2,1,2)
plot(t, part_med(:,2), 'r')
hold on
plot(t, part_mean(:,2), 'b')
plot(t, y_true, '--g');
ylabel('Y [m]')
hold off
legend('Median', 'Mean', 'True');

% Error
figure

subplot(2,1,1)
plot(t, (part_med(:,1) - x_true).^2, '-r')
hold on
plot(t, (part_mean(:,1) - x_true).^2, '-b')
hold off
ylabel('X Sqrd Err')
legend('Median', 'Mean');
title('Error');

subplot(2,1,2)
plot(t, (part_med(:,2) - y_true).^2, '-r')
hold on
plot(t, (part_mean(:,2) - y_true).^2, '-b')
hold off
ylabel('Y Sqrd Err')
legend('Median', 'Mean');
xlabel('Time [s]')

profile viewer


