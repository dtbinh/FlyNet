function [time,amcl,vicon] = parseAMCL(filename)

data = load(filename);

time = data(:,1);
amcl.x = data(:,2);
amcl.y = data(:,3);
amcl.psi = data(:,4);
vicon.x = data(:,5);
vicon.y = data(:,6);
vicon.psi = data(:,7);
end