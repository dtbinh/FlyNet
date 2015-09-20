function plot_sub2(x, y1, y2, xlab, ylab1, ylab2, title_str, new_fig)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot_sub3
% Austin Lillard
% Created: 04/07/2015
% Updated: 04/07/2015
% Purpose:
%		- To plot 2 items on a subplot
% 
% Input:
%		-x: x axis vector
%		-y1: y data, plot 1
%		-y2: y data, plot 2
%		-xlab: xlabel
%		-ylab1: ylabel, plot 1
%		-ylab2: ylabel, plot 2
%		-title_str: title string
%		-new_fig: 1 - Create new figure, 0 - Do not create new figure
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Create new figure
if new_fig == 1
	figure
end

% Plot 1
subplot(2,1,1)
plot(x, y1)
ylabel(ylab1,'Interpreter','latex','FontSize',12)
title(title_str,'Interpreter','latex','FontSize',12)

% Plot 2
subplot(2,1,2)
plot(x, y2)
ylabel(ylab2,'Interpreter','latex','FontSize',12)
xlabel(xlab,'Interpreter','latex','FontSize',12)

end