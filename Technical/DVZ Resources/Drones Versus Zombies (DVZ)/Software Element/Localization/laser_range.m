 function [z_meas, ind_x, ind_y, ind_pos_x, ind_pos_y] = laser_range(x, y, psi, map, map_res)

% laser_range
% Austin Lillard
% Created: 01/27/2015
% Updated: 01/27/2015
% Purpose:
%       - To calculate the distance measurement given by a laser range
%       finder in an occupancy grid.
% Inputs:
%       -x: x position in map [m]
%       -y: y position in map [m]
%       -psi: yaw angle [rad], +CCW, 0 aligned with x-axis
%       -map: occupancy grid matrix (mxn), 1 == occupied, 0 ==  unoccupied
%       -map_res: length of unit square in grid [m]
%
%       Example map:
%       y
%       ^
%       | 1 1 1 1 1 1 1 1
%       | 1 0 0 0 0 0 0 1
%       | 1 0 0 0 0 0 0 1      
%       | 1 0 0 1 1 1 0 1
%       | 1 1 1 1 0 1 1 1  
%       - - - - - - - - - -> x
% Outputs:
%       -z_meas: measured distance, returns -1 if quad in wall
%       -ind_x: x index of laser hit
%       -ind_y: y index of laser hit
%       -ind_pos_x: x index of position
%       -ind_pos_y: y index of position
            

%% Script example values

% clear all
% close all
% clc

% % Shoot beam to the right (psi = 0)
% % initial position (1, 1)
% % Square map
% 
% map = imread('map_example.png');
% map = mat2gray(map, [80 100]);
% map = imcomplement(map);

% map(map > 0) = 1;

% x = 10;
% y = 13;
% psi = pi/4;

% x = 1.5;
% y = 1;
% psi = pi/4;
% map = zeros(50);
% map(1, :) = ones(1, 50);
% map(end, :) = ones(1, 50);
% map(:, 1) = ones(50, 1);
% map(:, end) = ones(50,1);
% 
% map_res = 0.05;

%% Setup Map

% Map matrix dimensions
[m, n] = size(map);

% Set map origin
x_axis = map_res: map_res :map_res*n;       % positive index -> increase in x value
y_axis = map_res*m: -map_res :map_res;      % positive index -> decrease in y value


%% Setup Beam

% Maximum value the beam can return [m]
zmax = 7;

% Estimated points in unit grid space, assuming beam entry is perpendicular
% to the side of the square.
points_unit = 4;

% Discretization of the beam.  
% Spacing between discretized points [m]
beam_spacing = map_res/points_unit;

% Create beam
beam = 0: beam_spacing : zmax;


%% Find when beam hits the wall
    
% Calculate x and y positions of discretized beam in map coordinates
beam_x = x + beam*cos(psi);
beam_y = y + beam*sin(psi);

% Find indices in matrix that discretized beam has landed in
for ii = 1:length(beam)
    

   
    % Find which bin the beam sample lands in.  Similar to method used in
    % 1-D particle filter code.  Need a negative one for y axis because the
    % value is decreasing as the index increases.
    ind_x = length(x_axis((beam_x(ii) - x_axis ) >= 0));
    ind_y = length(y_axis(-1*(beam_y(ii) - y_axis ) >= 0));
    
    if ind_x <= 1 
        ind_x = 1;
    end
    if ind_y <=1
        ind_y = 1;
    end
    
    % Record initial position of beam, show location of quadcopter
    if ii == 1
       
        ind_pos_x = ind_x;
        ind_pos_y = ind_y;
        
    end
    
    % Check if grid unit is occupied or not
    occupancy = map(ind_y, ind_x);
    
    % If map grid space is occupied, exit loop and set laser range
    % measurement
    if occupancy == 1 && ii > 1
        z_meas = (ii - 1)*beam_spacing;
        break
    elseif occupancy == 1 && ii == 1
        z_meas = -1;
        break
    end
    
    
end

% Return max laser value if the beam never intersected an occupied space
if occupancy == 0
    z_meas = zmax;
end


%% Display as image

% Return data for imaging


% Make position and laser hit gray
% for nn = 1:5
%     for jj = 1:5
%       
%         map(ind_pos_y + nn - 3, ind_pos_x + jj - 3, 2) = 1;
%         map(ind_pos_y + nn - 3, ind_pos_x + jj - 3, 3) = 1;
%         
%         map(ind_y + nn - 3, ind_x + jj -3, 1) = 1;
%         
%         
%     end
% end
%  map(ind_pos_y, ind_pos_x) = map(ind_pos_y, ind_pos_x) + 0.5;
%  map(ind_y, ind_x, 1) = 0.5;
% 
% 
% % Change to walls = black and free = white
%  occ_grid = imcomplement(map);
% %occ_grid = map;
% 
% % Resize for ease of viewing
% occ_grid = imresize(occ_grid, 3, 'nearest');
% 
% % Show on screen
% figure 
% imshow(occ_grid)
    
   







