

%% Particle Filter - 1 Dimensional

% part_filter_1D
% Austin Lillard, Tyler King, DVZ
% Created: 01/21/2015
% Updated: 01/26/2015
% Purpose:
%   - Simple one dimensional particle filter implementation for
%   understanding concepts behind algorithm.
%% Clean up
clear all
close all
clc

%% Initialize

animate = 0;            % turn animation on and off

sigma_laser = 0.1;       %std dev of range measurement [m]

bias_laser = 0.05;		 %[m]

sigma_velocity = 1;      %std dev of velocity measurement [m/s]

bias_velocity = 0.1;	 %[m/s]

num_particles = 1000;     % number of particles being tracked

t = linspace(0, 100, 100)'; % time [s]

x0 = 1;                 % initial true position [m]

%% True position

movement = 3*sin(0.5*t);    % movement pattern of object [m]

velocity = 3*0.5*cos(0.5*t); % velocity patter of object [m/s]

% movement = 0.1*rand(length(t),1);

x_true = x0*ones(length(t), 1) + movement; % true position moves in sinusoidal pattern

%% Particles

% Create array
part_array = zeros(num_particles, 1);

% Initialize particles
part_array = 10*rand(size(part_array));

% Allocate Arrays
part_bin = zeros(num_particles, 1);
part_new = zeros(num_particles, 1);
part_med = zeros(length(t), 1);
part_mean = zeros(length(t), 1);
part_mode = zeros(length(t), 1);


% Run particle filter
for n = 2:length(t)
    
	velocity_meas = velocity(n) + randn * sigma_velocity + bias_velocity;
	
    % Update Particles from odometry info
    part_array(:, n) = part_array(:, n-1) + velocity_meas*(t(n)-t(n-1));
    
    % Add odometry noise
    part_array(:, n) = part_array(:, n) + sigma_velocity*randn(num_particles, 1);
    
    % Expected Measurement [m]
    part_meas = part_array(:, n);
    
    % Actual Measurement [m]
    true_meas = x_true(n) + sigma_laser*randn + bias_laser;
    
    % Calculate probabilities of each point
    part_prob = normpdf(part_meas, true_meas, sigma_laser);
    
    % Normalize to 1
    part_prob = part_prob./sum(part_prob);
    
	part_mode(n) = part_array(part_prob==max(part_prob),n);
	
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
       part_new(ii) = part_meas(bin_num); 
       
    end
    
    % Update particle array with new locations
    part_array(:, n) = part_new;
    
    % Median particle value
    part_med(n) = median(part_array(:,n));
    part_mean(n) = mean(part_array(:, n));
    
end

%% Plot Results

% Estimated position
figure
subplot(2,1,1)
plot(t, part_med, 'r')
hold on
plot(t, part_mean, 'b')
plot(t, part_mode, 'k');
plot(t, x_true, '--g');
hold off
ylabel('Est. Value [m]')
legend('Median', 'Mean', 'Mode', 'True');

% Error
subplot(2,1,2)
plot(t, (part_med - x_true).^2, '-r')
hold on
plot(t, (part_mean - x_true).^2, '-b')
plot(t, (part_mode - x_true).^2, '-k')
hold off
xlabel('Time [s]')
ylabel('Abs Sqrd Err')
legend('Median', 'Mean', 'Mode');
ylim([0 max((part_mean(10:end) - x_true(10:end)).^2)*1.1])
%% Animate Results

if animate == 1
    zero_arr1 = zeros(num_particles, length(t));
    zero_arr2 = zeros(length(t), 1);
    
    figure
    part_plot = part_array(:,1);
    true_plot = x_true(1);
    zero_plot1 = zero_arr1(:,1);
    zero_plot2 = zero_arr2(1);
    
    h = plot(part_plot, zero_plot1, '*r','XDataSource', 'part_plot', 'YDataSource', 'zero_plot1');
    hold on
    h2 = plot(true_plot, zero_plot2, 'og', 'XDataSource', 'true_plot', 'YDataSource', 'zero_plot2');

    hold off
    ylim([-1 1]);
    xlim([min(x_true) - 1,  max(x_true) + 1])
    
    for jj = 1:length(t)
        
        part_plot = part_array(:, jj);
        
        true_plot = x_true(jj);

        zero_plot1 = zero_arr1(:, jj);
        zero_plot2 = zero_arr2(jj);
    
        
        refreshdata(h, 'caller')
        refreshdata(h2, 'caller')
        
        drawnow
        pause(0.3)
        
        
    end
    
end





