function plot_3single(x, y1, y2, y3, xlab, ylab, title_str, leg1, leg2, leg3, new_fig)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot_3single
% Austin Lillard
% Created: 04/07/2015
% Updated: 04/07/2015
% Purpose:
%		- To plot 3 items on a single plot
% Input:
%		-x: x axis vector
%		-y1: y1 data
%		-y2: y2 data
%		-y3: y3 data
%		-xlab: xlabel
%		-ylab: ylabel, plot 1
%		-title_str: title string
%		-leg1: legend 1 string
%		-leg2: legend 2 string
%		-leg3: legend 3 string
%		-new_fig: 1 - Create new figure, 0 - Do not create new figure
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create new figure
if new_fig == 1
	figure
end

% Plot data
plot(x, y1, '-b')
hold on
plot(x, y2, '-r')
plot(x, y3, '-g')
xlabel(xlab,'Interpreter','latex','FontSize',12)
ylabel(ylab,'Interpreter','latex','FontSize',12)
title(title_str,'Interpreter','latex','FontSize',12)
legend(leg1, leg2, leg3)
