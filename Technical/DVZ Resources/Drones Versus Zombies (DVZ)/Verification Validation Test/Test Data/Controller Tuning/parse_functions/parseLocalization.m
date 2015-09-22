function [local] = parseLocalization(filename)

data = load(filename);

local.time = data(:,1) - data(1,1); % system time
local.amcl_x = data(:,2); % quad inertial x position from amcl
local.amcl_y = data(:,3); % quad inertial y position from amcl
local.amcl_z = data(:,4); % quad inertial z position from amcl
local.velx_body = data(:,5); % quad x velocity in body frame
local.vely_body = data(:,6); % quad y velocity in body frame
local.velz_map = data(:,7); % quad z velocity in map inertial frame
local.accx_body = data(:,8); % quad x acceleration in body frame
local.accy_body = data(:,9); % quad y acceleration in body frame
local.accz_map = data(:,10); % quad z acceleration in map inertial frame
local.psi_map = data(:,11); % quad yaw angle in map inertial frame
end