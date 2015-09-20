clear variables
close all
clc

showbode = 0;
showbandwidthinfo = 1;
showstep = 0;
showstepinfo = 1;
showpole = 0;

%% Controller Gains
defineGains
gains = readGains('current_gains.txt');

%% Transfer Functions
g = 9.81;
% Phi desired to Phi
TF_phi_num = [gains.Pa_phi*gains.Pr_phi gains.Pa_phi*gains.Pr_phi gains.Pa_phi*gains.Ir_phi];
TF_phi_den = [(1 + gains.Dr_phi) (gains.Pr_phi + gains.Pa_phi*gains.Pr_phi) (gains.Ir_phi + gains.Pa_phi*gains.Pr_phi) gains.Pa_phi*gains.Ir_phi];
TF_phi = tf(TF_phi_num,TF_phi_den);

% Theta desired to Theta
TF_theta_num = [gains.Pa_theta*gains.Pr_theta gains.Pa_theta*gains.Pr_theta gains.Pa_theta*gains.Ir_theta];
TF_theta_den = [(1 + gains.Dr_theta) (gains.Pr_theta + gains.Pa_theta*gains.Pr_theta) (gains.Ir_theta + gains.Pa_theta*gains.Pr_theta) gains.Pa_theta*gains.Ir_theta];
TF_theta = tf(TF_theta_num,TF_theta_den);

% Vx desired to Vx
TF_vx_num = [-g*gains.Dvx -g*gains.Pvx -g*gains.Ivx];
TF_vx_den = [(1 - g*gains.Dvx) -g*gains.Pvx -g*gains.Ivx];
TF_vx = tf(TF_vx_num,TF_vx_den);

% Vy desired to Vy
TF_vy_num = [g*gains.Dvy g*gains.Pvy g*gains.Ivy];
TF_vy_den = [(1 + g*gains.Dvy) g*gains.Pvy g*gains.Ivy];
TF_vy = tf(TF_vy_num,TF_vy_den);

% x desired to x
TF_x_num = [gains.Dx*gains.Dvy (gains.Dx*gains.Pvx + gains.Px*gains.Dvx) (gains.Px*gains.Pvx + gains.Ix*gains.Dvx + gains.Dx*gains.Ivx) (gains.Ix*gains.Pvx + gains.Px*gains.Ivx) gains.Ix*gains.Ivx];
TF_x_den = [(gains.Dx*gains.Dvx + gains.Dvx - (1/g)) (gains.Dx*gains.Pvx + gains.Px*gains.Dvx + gains.Pvx) (gains.Px*gains.Pvx + gains.Ix*gains.Dvx + gains.Dx*gains.Ivx + gains.Ivx) (gains.Ix*gains.Pvx + gains.Px*gains.Ivx) gains.Ix*gains.Ivx];
TF_x = tf(TF_x_num,TF_x_den);

% y desired to y
TF_y_num = [gains.Dy*gains.Dvy (gains.Dy*gains.Pvy + gains.Py*gains.Dvy) (gains.Py*gains.Pvy + gains.Iy*gains.Dvy + gains.Dy*gains.Ivy) (gains.Iy*gains.Pvy + gains.Py*gains.Ivy) gains.Iy*gains.Ivy];
TF_y_den = [(gains.Dy*gains.Dvy + gains.Dvy + (1/g)) (gains.Dy*gains.Pvy + gains.Py*gains.Dvy + gains.Pvy) (gains.Py*gains.Pvy + gains.Iy*gains.Dvy + gains.Dy*gains.Ivy + gains.Ivy) (gains.Iy*gains.Pvy + gains.Py*gains.Ivy) gains.Iy*gains.Ivy];
TF_y = tf(TF_y_num,TF_y_den);

%% Plotting
if showbode == 1
    % Phi desired to Phi
    figure
    hold on
    grid on
    bode(TF_phi)
    title('Phi/Phi_{des} Frequency Response')
    
    % Theta desired to Theta
    figure
    bode(TF_theta)
    title('Theta/Theta_{des} Frequency Response')
        
    % Vx desired to Vx
    figure
    bode(TF_vx)
    title('Vx/Vx_{des} Frequency Response')
        
    % Vy desired to Vy
    figure
    bode(TF_vy)
    title('Vy/Vy_{des} Frequency Response')
    
    % x desired to x
    figure
    bode(TF_x)
    title('x/x_{des} Frequency Response')
    
    % y desired to y
    figure
    bode(TF_y)
    title('y/y_{des} Frequency Response')
end

if showbandwidthinfo == 1
    phi_bandwidth = bandwidth(TF_phi)/(2*pi);
    fprintf(['Phi/Phi_des Bandwidth = ' num2str(phi_bandwidth) ' Hz\n'])
    theta_bandwidth = bandwidth(TF_theta)/(2*pi);
    fprintf(['Theta/Theta_des Bandwidth = ' num2str(theta_bandwidth) ' Hz\n'])
    vx_bandwidth = bandwidth(TF_vx)/(2*pi);
    fprintf(['Vx/Vx_des Bandwidth = ' num2str(vx_bandwidth) ' Hz\n'])
    vy_bandwidth = bandwidth(TF_vy)/(2*pi);
    fprintf(['Vy/Vy_des Bandwidth = ' num2str(vy_bandwidth) ' Hz\n'])
    x_bandwidth = bandwidth(TF_x)/(2*pi);
    fprintf(['x/x_des Bandwidth = ' num2str(x_bandwidth) ' Hz\n'])
    y_bandwidth = bandwidth(TF_y)/(2*pi);
    fprintf(['y/y_des Bandwidth = ' num2str(y_bandwidth) ' Hz\n'])
end

if showstep == 1
    % phi desired to phi
    figure
    step(TF_phi)
    title('Phi/Phi_{des} Step Response')
    
    % theta desired to theta
    figure
    step(TF_theta)
    title('Theta/Theta_{des} Step Response')
    
    % Vx desired to Vx
    figure
    step(TF_vx)
    title('Vx/Vx_{des} Step Response')
    
    % Vy desired to Vy
    figure
    step(TF_vy)
    title('Vy/Vy_{des} Step Response')
    
    % x desired to x
    figure
    step(TF_x)
    title('x/x_{des} Step Response')
    
    % y desired to y
    figure
    step(TF_y)
    title('y/y_{des} Step Response')
    
    % Plot phi, Vy, and y position on one plot
    t = 0:0.1:10;
    [step_phi,~] = step(TF_phi,t);
    [step_vy,~] = step(TF_vy,t);
    [step_y,~] = step(TF_y,t);
    figure
    hold on
    grid on
    plot(t,step_phi,'r','LineWidth',2)
    plot(t,step_vy,'b','LineWidth',2)
    plot(t,step_y,'g','LineWidth',2)
    xlabel('Time [s]')
    ylabel('Response')
    title('Roll, Velocity Y, Position Y Step Response')
    legend('Roll','Vel_y','Y','Location','Best')
    
    % Plot theta, Vx, and x position on one plot
    t = 0:0.1:10;
    [step_theta,~] = step(TF_theta,t);
    [step_vx,~] = step(TF_vx,t);
    [step_x,~] = step(TF_x,t);
    figure
    hold on
    grid on
    plot(t,step_theta,'r','LineWidth',2)
    plot(t,step_vx,'b','LineWidth',2)
    plot(t,step_x,'g','LineWidth',2)
    xlabel('Time [s]')
    ylabel('Response')
    title('Pitch, Velocity X, Position X Step Response')
    legend('Pitch','Vel_x','X','Location','Best')
end

if showstepinfo == 1
    TF_phi_info = stepinfo(TF_phi);
    TF_theta_info = stepinfo(TF_theta);
    TF_vx_info = stepinfo(TF_vx);
    TF_vy_info = stepinfo(TF_vy);
    TF_x_info = stepinfo(TF_x);
    TF_y_info = stepinfo(TF_y);
    fprintf('___Parameter___|___TF_x___|___TF_y___|___TF_vx___|___TF_vy___|___TF_phi___|___TF_theta___|\n');
    fprintf('       RiseTime| %1.5f  | %1.5f  | %1.5f   | %1.5f   | %1.5f    | %1.5f      |\n',...
            TF_x_info.RiseTime, TF_y_info.RiseTime, TF_vx_info.RiseTime, TF_vy_info.RiseTime, TF_phi_info.RiseTime, TF_theta_info.RiseTime);
    fprintf('   SettlingTime| %1.5f  | %1.5f  | %1.5f   | %1.5f   | %1.5f    | %1.5f      |\n',...
            TF_x_info.SettlingTime, TF_y_info.SettlingTime, TF_vx_info.SettlingTime, TF_vy_info.SettlingTime, TF_phi_info.SettlingTime, TF_theta_info.SettlingTime);
    fprintf('    SettlingMin| %1.5f  | %1.5f  | %1.5f   | %1.5f   | %1.5f    | %1.5f      |\n',...
            TF_x_info.SettlingMin, TF_y_info.SettlingMin, TF_vx_info.SettlingMin, TF_vy_info.SettlingMin, TF_phi_info.SettlingMin, TF_theta_info.SettlingMin);
    fprintf('    SettlingMax| %1.5f  | %1.5f  | %1.5f   | %1.5f   | %1.5f    | %1.5f      |\n',...
            TF_x_info.SettlingMax, TF_y_info.SettlingMax, TF_vx_info.SettlingMax, TF_vy_info.SettlingMax, TF_phi_info.SettlingMax, TF_theta_info.SettlingMax);
    fprintf('      Overshoot| %1.5f  | %1.5f  | %1.5f   | %1.5f  | %1.5f    | %1.5f      |\n',...
            TF_x_info.Overshoot, TF_y_info.Overshoot, TF_vx_info.Overshoot, TF_vy_info.Overshoot, TF_phi_info.Overshoot, TF_theta_info.Overshoot);
    fprintf('     Undershoot| %1.5f | %1.5f | %1.5f  | %1.5f  | %1.5f   | %1.5f     |\n',...
            TF_x_info.Undershoot, TF_y_info.Undershoot, TF_vx_info.Undershoot, TF_vy_info.Undershoot, TF_phi_info.Undershoot, TF_theta_info.Undershoot);
    fprintf('           Peak| %1.5f  | %1.5f  | %1.5f   | %1.5f   | %1.5f    | %1.5f      |\n',...
            TF_x_info.Peak, TF_y_info.Peak, TF_vx_info.Peak, TF_vy_info.Peak, TF_phi_info.Peak, TF_theta_info.Peak);
    fprintf('       PeakTime| %1.5f | %1.5f  | %1.5f   | %1.5f   | %1.5f    | %1.5f      |\n',...
            TF_x_info.PeakTime, TF_y_info.PeakTime, TF_vx_info.PeakTime, TF_vy_info.PeakTime, TF_phi_info.PeakTime, TF_theta_info.PeakTime);
end

if showpole == 1
    % phi desired to phi
    figure
    pzplot(TF_phi)
    title('Phi/Phi_{des} Pole-Zero Plot')
    
    % theta desired to theta
    figure
    pzplot(TF_theta)
    title('Theta/Theta_{des} Pole-Zero Plot')
    
    % Vx desired to Vx
    figure
    pzplot(TF_vx)
    title('Vx/Vx_{des} Pole-Zero Plot')
    
    % Vy desired to Vy
    figure
    pzplot(TF_vy)
    title('Vy/Vy_{des} Pole-Zero Plot')
    
    % x desired to x
    figure
    pzplot(TF_x)
    title('x/x_{des} Pole-Zero Plot')
    
    % y desired to y
    figure
    pzplot(TF_y)
    title('y/y_{des} Pole-Zero Plot')
end