%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller Tuning Post Processing
% Project DVZ
% Programmer: Mark Sakaguchi
% Created: 2/20/2015
% Updated: 2/23/2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Cleansing
clear all
close all
clc

%% Add functions and data paths
addpath('parse_functions');
addpath('plot_functions');
addpath('Position_Controller_Tuning_03_03_15');
addpath('px4flow_tuning_03_11_15');

%% Load data
% test_num = 17;

% % file_alt = ['alt_test',num2str(test_num),'.txt'];
% file_vel = ['vel_test',num2str(test_num),'.txt'];
% file_pos = ['pos_test',num2str(test_num),'.txt'];
% % file_rc = ['rc_test',num2str(test_num),'.txt'];
%
% % alt_cont = parseAltController(file_alt);
% vel_cont = parseVelocityController(file_vel);
% pos_cont = parsePositionController(file_pos);
% rc = parseRC(file_rc);

%% Plot data
% plotAltController(alt_cont);
% plotVelocityController(vel_cont);
% plotPositionController(pos_cont);
% plotRC(rc);

%% Plot Altitude Stuff
test_num = 1;

file_rc = ['flow_test',num2str(test_num),'.txt'];
rc = parseRC(file_rc);

ind_trash = find(rc.time<9.4);
rc.time(ind_trash) = [];
rc.z(ind_trash) = [];

time_drop = 0;
vel_drop = 0;
last_meas = 0;
figure
hold on
grid on
for i = 1:length(rc.z)
    if i == 1
        filtered_z(i) = rc.z(i);
        vel_standard = 0;
    else
        delta_t = rc.time(i)-rc.time(i-1);
        
        if rc.z(i) == 0
            time_drop = time_drop+delta_t;
            
            if rc.z(i) == 0 && rc.z(i-1) == 0
                filtered_z(i) = filtered_z(i-1)+(vel_standard*delta_t)*(delta_t/time_drop);
            else
                filtered_z(i) = rc.z(i-1)+(vel_standard*delta_t);
            end    
            delta_pos = filtered_z(i)-filtered_z(i-1);
        else
            
            filtered_z(i) = rc.z(i);
            delta_pos = filtered_z(i)-filtered_z(i-1);
            if time_drop ~= 0
                if last_meas ~=0
                    vel_drop = (rc.z(i)-last_meas)/time_drop;
                end
            end
            
            if vel_drop ==0
                vel_standard = delta_pos/delta_t;
            else
                vel_standard = vel_drop;
                vel_drop = 0;
            end
            
            last_meas = rc.z(i);
            filtered_z(i) = rc.z(i);
            time_drop = 0;
        end
    end
    plot(rc.time(i),filtered_z(i),'bo')
    plot(rc.time(i),rc.z(i),'rx')
end
%% Marks
% prev_ind = 1;
% cur_ind = 1;
% for i = 1:length(rc.z)
%     filtered_z(i) = rc.z(i);
%     if i > 1
%         delta_t = rc.time(i)-rc.time(i-1);
%         delta_pos = rc.z(i)-rc.z(i-1);
%         vel_standard(i) = delta_ps/delta_t;
%         if rc.z(i) == 0
%
%             filtered_z(i) = rc.z(i-1);
%             if rc.z(i) == 0 && rc.z(i-1) == 0
%                 filtered_z(i) = filtered_z(i-1);
%             end
%         end
%     end
% end
%%
ind_orig = find(rc.z == 0);
ind_new = find(filtered_z == 0);

figure
hold on
grid on
for j = 1:length(rc.z)
    plot(rc.time(j),filtered_z(j),'bo')
    plot(rc.time(j),rc.z(j),'rx')
    %     waitforbuttonpress()
    pause(0.01)
end
xlabel('Time [s]')
ylabel('Filtered Alt [m]')
%
% figure
% plot(rc.time,rc.z,'b'),grid
% xlabel('Time [s]')
% ylabel('Altitude [m]')

% figure
% subplot(2,1,1)
% plot(rc.time,rc.z)
% title('Altitude from Px4flow')
% ylabel('Altitude (m)')
% subplot(2,1,2)
% plot(rc.time,rc.throttle)
% title('PWM From Edward')
% ylabel('PWM')
% ylabel('Time (s)')

