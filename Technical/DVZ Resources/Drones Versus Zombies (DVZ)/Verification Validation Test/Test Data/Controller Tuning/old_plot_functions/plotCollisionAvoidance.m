function plotCollisionAvoidance(ca)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotAltController.m
% Programmer: Mark Sakaguchi
% Created: 4/1/15
% Updated: 4/1/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%
% Outputs:
%
% Purpose:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on
grid on
plot(ca.time,ca.min_dist,'k--')
plot(ca.time,ca.min_range,'b')
xlabel('Time [s]')
ylabel('Distance [m]')
title('Minimum Range')
legend('min dist','min range','location','best')

figure
subplot(311)
plot(ca.time,ca.angle.*(180/pi),'r'),grid
ylabel('Min Range Angle [deg]')
title('Collision Avoidance Commands')
subplot(312)
plot(ca.time,ca.target_velx_body,'r'),grid
ylabel('Vel_{x,body} [m/s]')
subplot(313)
plot(ca.time,ca.target_vely_body,'r'),grid
xlabel('Time [s]')
ylabel('Vel_{y,body} [m/s]')

angle = linspace(mean(ca.angle_min), mean(ca.angle_max), size(ca.ranges,2));
figure
hold on
grid on
for i = 1:length(ca.time)
	bin = (ca.angle(i) - mean(ca.angle_min))/mean(ca.angle_inc);
	plot_arc(-120*(pi/180),120*(pi/180),0,0,mean(ca.min_dist))
	[h1,h2] = plot_heading(ca.angle(i),ca.min_range(i));
	axis square
	xlim([-1 1])
	ylim([-1 1])
	for n = 1:size(ca.ranges,2)
		dist(n) = ca.ranges(i,n);
	end
	[x,y] = pol2cart(angle,dist);
	p1 = plot(x,y,'k','LineWidth',2);
	view(-90,90)
	pause(0.01)
	if i ~= length(ca.time)
		delete(h1)
		delete(h2)
		delete(p1)
	end
end

end