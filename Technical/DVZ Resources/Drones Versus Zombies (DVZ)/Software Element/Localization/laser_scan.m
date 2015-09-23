function [z_array] = laser_scan(x, y, psi, num_beams, map, map_res)

% laser_scan
% Austin Lillard
% Created: 01/27/2015
% Updated: 01/27/2015
% Purpose:
%       - To calculate an array of laser ranges corresponding to a
%       subsampled portion of the hokuyo scan data.
% Inputs:
%       -x: x location of the vehicle [m]
%       -y: y location of the vehicle [m]
%       -psi: yaw angle of quad [rad]
%       -num_beams: how many beams to use in scan
% Outputs:
%       -z_meas_array: array of range values from evenly spaced laser scan
%       angles.

% %% For script testing
% 
% % Load map
% map = imread('map_example.png');
% map = mat2gray(map, [80 100]);
% map = imcomplement(map);
% 
% map(map > 0) = 1;
% 
% map_res = 0.05;
% 
% x = 10;
% y = 13;
% psi = 0;
% 
% num_beams = 30;



%% Setup

% Minimum laser scan angle [rad]
alpha_min = -135*pi/180;

% Maximum laser scan angle [rad]
alpha_max = 135*pi/180;

% Create equally spaced scan angles, body frame
body_angles = linspace(alpha_min, alpha_max, num_beams);

% Put into map frame
map_angles = psi + body_angles;

%% Calculate ranges

% Call the laser_range function for each beam angle
for ii = 1 : num_beams
    
   [z_array(ii), ind_x(ii), ind_y(ii), ind_pos_x, ind_pos_y] = laser_range_opt(x, y, map_angles(ii), map, map_res);
    
end


% %% Display results
% % Make position and laser hit gray
% 
% for nn = 1:5
%     for jj = 1:5
%         
%         map(ind_pos_y + nn - 3, ind_pos_x + jj - 3, 2) = 1;
%         map(ind_pos_y + nn - 3, ind_pos_x + jj - 3, 3) = 1;
%         
%         for ii = 1:num_beams
%              map(ind_y(ii) + nn - 3, ind_x(ii) + jj -3, 1) = 1;
%         end
%         
%     end
% end
% % map(ind_pos_y, ind_pos_x) = map(ind_pos_y, ind_pos_x) + 0.5;
% 
% 
% % Change to walls = black and free = white
%  occ_grid = imcomplement(map);
% %occ_grid = map;
% 
% % Resize for ease of viewing
% occ_grid = imresize(occ_grid, 1, 'nearest');
% 
% % Show on screen
% figure 
% imshow(occ_grid)






