function [batt] = parseBattery(filename)

data = load(filename);

batt.time = data(:,1) - data(1,1); % system time
batt.voltage = data(:,2)./1000; % quad voltage
batt.current = data(:,3)./10./1000; % quad current
batt.batt_remaining = data(:,4); % quad percentage of battery remaining

end