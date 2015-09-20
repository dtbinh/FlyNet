function plot_single(x, y, xlab, ylab, title_str, new_fig)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot_single
% Austin Lillard
% Created: 04/07/2015
% Updated: 04/07/2015
% Purpose:
%		- To plot a single item
% 
% Input: 
%		-x: x axis vector
%		-y: y data
%		-xlab: x label
%		-ylab: y label
%		-title_str: title string
%		-new_fig: 1 - Create new figure, 0 - Do not create new figure
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create new figure
if new_fig == 1
	figure
end

% Plot
plot(x, y)
xlabel(xlab,'Interpreter','latex','FontSize',12)
ylabel(ylab,'Interpreter','latex','FontSize',12)
title(title_str,'Interpreter','latex','FontSize',12)
