function [pos] = integrateVelocity(time,vel,pos0)

%deltat = time(2) - time(1);
% pos = pos0 + cumtrapz(time,vel);
% vel = vel - mean(vel);
% vel = detrend(vel);

pos = cumtrapz(time,vel);

% for i = 1:length(time)-1
%     deltat = (time(i+1) - time(i));
%     pos(i) = vel(i)*deltat;
% end
end