function [h1,h2] = plot_heading(pose,scaling)
psi = pose(3);

% Define world unit vectors
world_x = [1; 0];
world_y = [0; 1];

% Rotate world unit vectors to local vectors
R = [-sin(psi), cos(psi);...
     cos(psi), sin(psi)];
local_x = R*world_x;
local_y = R*world_y;

% Normalize local vectors to local unit vectors
local_x = local_x/norm(local_x)*scaling;
local_y = local_y/norm(local_y)*scaling;

h1 = quiver(pose(1),pose(2),local_x(1),local_x(2),0.25,'r','LineWidth',2);
h2 = quiver(pose(1),pose(2),local_y(1),local_y(2),0.25,'b','LineWidth',2);

end