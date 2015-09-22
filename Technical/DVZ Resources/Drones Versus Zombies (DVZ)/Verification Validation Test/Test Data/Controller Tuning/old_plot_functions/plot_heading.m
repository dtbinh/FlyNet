function [h1,h2] = plot_heading(psi,min_dist)
angle_vec = [cos(psi), sin(psi)];
vel_body_vec = [cos(psi+pi), sin(psi+pi)];

h1 = quiver(0,0,angle_vec(1),angle_vec(2),min_dist,'r','LineWidth',2);
h2 = quiver(0,0,vel_body_vec(1),vel_body_vec(2),0.1/min_dist,'g','LineWidth',2);

end