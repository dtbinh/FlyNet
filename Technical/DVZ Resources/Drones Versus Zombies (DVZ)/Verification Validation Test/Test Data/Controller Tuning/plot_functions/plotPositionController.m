function plotPositionController(pos_cont)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotPositionController.m
% Programmer: Mark Sakaguchi
% Created: 2/23/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   pos_cont - position structured variable containing data about position
%              controller.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the position controller structured
%    variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
one_plot = 0;
separate_plots = 1;
error_plots = 0;
plot_2d = 1;

if one_plot == 1
    figure
    subplot(2,2,1)
    hold on
    grid on
    plot(pos_cont.time,pos_cont.target_x,'b')
    plot(pos_cont.time,pos_cont.x,'r')
    xlabel('Time [s]')
    ylabel('X Position [m]')
    title('Inertial X Position')
    legend('X_{I,des}','X_{I}','Location','Best')
    
    subplot(2,2,3)
    hold on
    grid on
    plot(pos_cont.time,pos_cont.target_y,'b')
    plot(pos_cont.time,pos_cont.y,'r')
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    title('Inertial Y Position')
    legend('Y_{I,des}','Y_{I}','Location','Best')
    
    subplot(2,2,2)
    hold on
    grid on
    plot(pos_cont.time,pos_cont.rcx_p,'r')
    plot(pos_cont.time,pos_cont.rcx_i,'g')
    plot(pos_cont.time,pos_cont.rcx_d,'b')
    xlabel('Time [s]')
    ylabel('Inertial Velx Input [m/s]')
    title('Inertial X Velocity Input [m/s]')
    legend('P','I','D','Location','Best')
    
    subplot(2,2,4)
    hold on
    grid on
    plot(pos_cont.time,pos_cont.rcy_p,'r')
    plot(pos_cont.time,pos_cont.rcy_i,'g')
    plot(pos_cont.time,pos_cont.rcy_d,'b')
    xlabel('Time [s]')
    ylabel('Inertial Vely Input [m/s]')
    title('Inertial Y Velocity Input [m/s]')
    legend('P','I','D','Location','Best')
end
if separate_plots == 1
    figure
    hold on
    grid on
    plot(pos_cont.time,pos_cont.target_x,'b')
    plot(pos_cont.time,pos_cont.x,'r')
    xlabel('Time [s]')
    ylabel('X Position [m]')
    title('Inertial X Position')
    legend('X_{I,des}','X_{I}','Location','Best')
    
    figure
    hold on
    grid on
    plot(pos_cont.time,pos_cont.target_y,'b')
    plot(pos_cont.time,pos_cont.y,'r')
    xlabel('Time [s]')
    ylabel('Y Position [m]')
    title('Inertial Y Position')
    legend('Y_{I,des}','Y_{I}','Location','Best')
end
if error_plots == 1
    figure
    plot(pos_cont.time,(pos_cont.target_x - pos_cont.x).^2,'r'),grid
    xlabel('Time [s]')
    ylabel('X Position Error^2')
    title('Inertial X Position Error^2')
    
    figure
    plot(pos_cont.time,(pos_cont.target_y - pos_cont.y).^2,'r'),grid
    xlabel('Time [s]')
    ylabel('Y Position Error^2')
    title('Inertial Y Position Error^2')
end
if plot_2d == 1
    c1_center = [-1.5, -1.5];
    r1 = 0.1;
    c2_center = [-1.5, 1.5];
    r2 = 0.1;
    c3_center = [1.5, 1.5];
    r3 = 0.1;
    c4_center = [1.5, -1.5];
    r4 = 0.1;
    
    figure
    hold on
    grid on
    plot(pos_cont.x,pos_cont.y,'b')
    plot(pos_cont.target_x,pos_cont.target_y,'r')
    viscircles(c1_center,r1,'EdgeColor','k');
    viscircles(c2_center,r2,'EdgeColor','k');
    viscircles(c3_center,r3,'EdgeColor','k');
    viscircles(c4_center,r4,'EdgeColor','k');
    xlabel('X [m]')
    ylabel('Y [m]')
    title('Inertial X,Y Position')
    legend('X_{I},Y_{I}','X_{I,des},Y_{I,des}')
    axis equal
end
fprintf(['Mean Inertial X Position Error = ' num2str(mean(pos_cont.target_x - pos_cont.x)) ' [m]\n'])
fprintf(['Mean Inertial Y Position Error = ' num2str(mean(pos_cont.target_y - pos_cont.y)) ' [m]\n'])

end