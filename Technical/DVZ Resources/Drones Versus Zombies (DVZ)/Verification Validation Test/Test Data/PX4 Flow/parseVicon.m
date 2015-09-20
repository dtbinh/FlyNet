function [vicon] = parseVicon(filename)

data = load(filename);
vicon.frame = data(:,1);
vicon.x = data(:,2);    % [mm]
vicon.y = data(:,3);    % [mm]
vicon.z = data(:,4);    % [mm]
vicon.rotx = data(:,5);
vicon.roty = data(:,6);
vicon.rotz = data(:,7);

end