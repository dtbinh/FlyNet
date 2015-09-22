function plotVelocityController(vel_cont)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotVelocityController.m
% Programmer: Mark Sakaguchi
% Created: 2/23/15
% Updated: 3/29/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:
%   vel_cont - velocity structured variable containing data about velocity
%              controller.
%
% Outputs:
%
% Purpose:
%    Plot the different fields of the velocity controller structured
%    variable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
one_plot = 0;
separate_plots = 0;
error_plots = 0;

if one_plot == 1
    figure
    subplot(2,3,1)
    hold on
    grid on
    plot(vel_cont.time,vel_cont.target_velx_inertial,'b')
    plot(vel_cont.time,vel_cont.velx_inertial,'r')
    xlabel('Time [s]')
    ylabel('Inertial X Velocity [m/s]')
    title('Inertial X Velocity')
    legend('Velx_{I,des}','Velx_{I}','Location','Best')
    
    subplot(2,3,4)
    hold on
    grid on
    plot(vel_cont.time,vel_cont.target_vely_inertial,'b')
    plot(vel_cont.time,vel_cont.vely_inertial,'r')
    xlabel('Time [s]')
    ylabel('Inertial Y Velocity [m/s]')
    title('Inertial Y Velocity')
    legend('Vely_{I,des}','Vely_{I}','Location','Best')
    
    % Plot pitch controller PID contributions on same plot
    subplot(2,3,2)
    hold on
    grid on
    plot(vel_cont.time,vel_cont.rcx_p.*(180/pi),'r')
    plot(vel_cont.time,vel_cont.rcx_i.*(180/pi),'g')
    plot(vel_cont.time,vel_cont.rcx_d.*(180/pi),'b')
    xlabel('Time [s]')
    ylabel('Pitch [deg]')
    title('PID Pitch Angle Contribution')
    legend('P','I','D','Location','Best')
    
    % Plot pitch controller PID contributions on same plot
    subplot(2,3,5)
    hold on
    grid on
    plot(vel_cont.time,vel_cont.rcy_p.*(180/pi),'r')
    plot(vel_cont.time,vel_cont.rcy_i.*(180/pi),'g')
    plot(vel_cont.time,vel_cont.rcy_d.*(180/pi),'b')
    xlabel('Time [s]')
    ylabel('Roll [deg]')
    title('PID Roll Angle Contribution')
    legend('P','I','D','Location','Best')
    
    subplot(2,3,3)
    hold on
    grid on
    plot(vel_cont.time,vel_cont.pitch_angle_send,'b')
    plot(vel_cont.time,vel_cont.theta.*(180/pi),'r')
    xlabel('Time [s]')
    ylabel('Pitch Angle [deg]')
    title('Pitch Output of Velocity Controller')
    legend('Pitch_{des}','Pitch')
    
    subplot(2,3,6)
    hold on
    grid on
    plot(vel_cont.time,vel_cont.roll_angle_send,'b')
    plot(vel_cont.time,vel_cont.phi.*(180/pi),'r')
    xlabel('Time [s]')
    ylabel('Roll Angle [deg]')
    title('Roll Output of Velocity Controller')
    legend('Roll_{des}','Roll')
end
if separate_plots == 1
    figure
    hold on
    grid on
    plot(vel_cont.time,vel_cont.target_velx_inertial,'b')
    plot(vel_cont.time,vel_cont.velx_inertial,'r')
    xlabel('Time [s]')
    ylabel('Inertial X Velocity [m/s]')
    title('Inertial X Velocity')
    legend('Velx_{I,des}','Velx_{I}','Location','Best')
    
    figure
    hold on
    grid on
    plot(vel_cont.time,vel_cont.target_vely_inertial,'b')
    plot(vel_cont.time,vel_cont.vely_inertial,'r')
    xlabel('Time [s]')
    ylabel('Inertial Y Velocity [m/s]')
    title('Inertial Y Velocity')
    legend('Vely_{I,des}','Vely_{I}','Location','Best')
    
    figure
    hold on
    grid on
    plot(vel_cont.time,vel_cont.roll_angle_send,'b')
    plot(vel_cont.time,vel_cont.phi*(180/pi),'r')
    xlabel('Time [s]')
    ylabel('Roll Angle [deg]')
    title('Pixhawk Roll Controller')
    legend('Roll_{des}','Roll')
    
    figure
    hold on
    grid on
    plot(vel_cont.time,vel_cont.pitch_angle_send,'b')
    plot(vel_cont.time,vel_cont.theta*(180/pi),'r')
    xlabel('Time [s]')
    ylabel('Pitch Angle [deg]')
    title('Pixhawk Pitch Controller')
    legend('Pitch_{des}','Pitch')
end
if error_plots == 1
    figure
    plot(vel_cont.time,(vel_cont.target_velx_inertial - vel_cont.velx_inertial).^2,'r'),grid
    xlabel('Time [s]')
    ylabel('Inertial Velx Error^2')
    title('Inertial X Velocity Error^2')
    
    figure
    plot(vel_cont.time,(vel_cont.target_vely_inertial - vel_cont.vely_inertial).^2,'r'),grid
    xlabel('Time [s]')
    ylabel('Inertial Vely Error^2')
    title('Inertial Y Velocity Error^2')
    
    figure
    plot(vel_cont.time,(vel_cont.roll_angle_send - vel_cont.phi*(180/pi)).^2,'r'),grid
    xlabel('Time [s]')
    ylabel('Roll Angle Error^2')
    title('Roll Angle Error^2')
    
    figure
    plot(vel_cont.time,(vel_cont.pitch_angle_send - vel_cont.theta*(180/pi)).^2,'r'),grid
    xlabel('Time [s]')
    ylabel('Pitch Angle Error^2')
    title('Pitch Angle Error^2')
end

fprintf(['Mean Inertial X Velocity Error = ' num2str(mean(vel_cont.target_velx_inertial - vel_cont.velx_inertial)) ' [m/s]\n'])
fprintf(['Mean Inertial Y Velocity Error = ' num2str(mean(vel_cont.target_vely_inertial - vel_cont.vely_inertial)) ' [m/s]\n'])
fprintf(['Mean Roll Angle Error = ' num2str(mean(vel_cont.roll_angle_send - vel_cont.phi*(180/pi))) ' [deg]\n'])
fprintf(['Mean Pitch Angle Error = ' num2str(mean(vel_cont.pitch_angle_send - vel_cont.theta*(180/pi))) ' [deg]\n'])    
    
end