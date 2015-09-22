function [xy_odom] = RotationTester(xy_body,psi)
R1 = [cosd(psi) -sind(psi);...
     -sind(psi) -cosd(psi)];

xy_odom = R1*xy_body;
end

