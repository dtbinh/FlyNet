function [flow] = parseFlow(filename)

data = load(filename);

flow.time = data(:,1) - data(1,1); % system time
flow.alt = data(:,2); % altitude from px4flow
flow.quality = data(:,3); % quality from px4flow
flow.velx = data(:,4); % x velocity from px4flow
flow.vely = data(:,5); % y velocity from px4flow

end