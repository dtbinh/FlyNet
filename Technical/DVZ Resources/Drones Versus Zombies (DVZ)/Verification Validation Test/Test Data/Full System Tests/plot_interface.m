function varargout = plot_interface(varargin)
% PLOT_INTERFACE MATLAB code for plot_interface.fig
%      PLOT_INTERFACE, by itself, creates a new PLOT_INTERFACE or raises the existing
%      singleton*.
%
%      H = PLOT_INTERFACE returns the handle to a new PLOT_INTERFACE or the handle to
%      the existing singleton*.
%
%      PLOT_INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PLOT_INTERFACE.M with the given input arguments.
%
%      PLOT_INTERFACE('Property','Value',...) creates a new PLOT_INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before plot_interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to plot_interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help plot_interface

% Last Modified by GUIDE v2.5 24-Apr-2015 10:44:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @plot_interface_OpeningFcn, ...
    'gui_OutputFcn',  @plot_interface_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before plot_interface is made visible.
function plot_interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to plot_interface (see VARARGIN)

% Choose default command line output for plot_interface
handles.output = hObject;
fprintf('Opening\n')
% Handle of gui
fig_open = findall(0, 'type', 'figure');
gui_handle = fig_open(1);
setappdata(gui_handle, 'IgnoreCloseAll', 1);

% Initialize plot flags to 0
handles.vicon_position_plot = 0;
handles.pixhawk_attitude_plot = 0;
handles.amcl_pose_plot = 0;
handles.altitude_controller_plot = 0;
handles.velocity_controller_plot = 0;
handles.yaw_controller_plot = 0;
handles.position_controller_plot = 0;
handles.rc_handset_pwm_plot = 0;
handles.pixhawk_battery_plot = 0;
handles.pixhawk_raw_imu_plot = 0;
handles.localization_sub_plot = 0;
handles.px4flow_sub_plot = 0;
handles.collision_avoidance_plot = 0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes plot_interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = plot_interface_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in vicon_position.
function vicon_position_Callback(hObject, eventdata, handles)
% hObject    handle to vicon_position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of vicon_position

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.vicon_position_plot = 1;
else
    handles.vicon_position_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in pixhawk_attitude.
function pixhawk_attitude_Callback(hObject, eventdata, handles)
% hObject    handle to pixhawk_attitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pixhawk_attitude

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.pixhawk_attitude_plot = 1;
else
    handles.pixhawk_attitude_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in amcl_pose.
function amcl_pose_Callback(hObject, eventdata, handles)
% hObject    handle to amcl_pose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of amcl_pose

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.amcl_pose_plot = 1;
else
    handles.amcl_pose_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in altitude_controller.
function altitude_controller_Callback(hObject, eventdata, handles)
% hObject    handle to altitude_controller (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of altitude_controller

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.altitude_controller_plot = 1;
else
    handles.altitude_controller_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in velocity_controller.
function velocity_controller_Callback(hObject, eventdata, handles)
% hObject    handle to velocity_controller (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of velocity_controller

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.velocity_controller_plot = 1;
else
    handles.velocity_controller_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in yaw_controller.
function yaw_controller_Callback(hObject, eventdata, handles)
% hObject    handle to yaw_controller (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of yaw_controller

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.yaw_controller_plot = 1;
else
    handles.yaw_controller_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in position_controller.
function position_controller_Callback(hObject, eventdata, handles)
% hObject    handle to position_controller (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of position_controller

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.position_controller_plot = 1;
else
    handles.position_controller_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in rc_handset_pwm.
function rc_handset_pwm_Callback(hObject, eventdata, handles)
% hObject    handle to rc_handset_pwm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rc_handset_pwm

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.rc_handset_pwm_plot = 1;
else
    handles.rc_handset_pwm_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)



function test_num_Callback(hObject, eventdata, handles)
% hObject    handle to test_num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of test_num as text
%        str2double(get(hObject,'String')) returns contents of test_num as a double

handles.num = get(hObject, 'String');

guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function test_num_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', '1');

handles.num = get(hObject, 'String');

guidata(hObject, handles)


% --- Executes on button press in pixhawk_battery.
function pixhawk_battery_Callback(hObject, eventdata, handles)
% hObject    handle to pixhawk_battery (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pixhawk_battery

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.pixhawk_battery_plot = 1;
else
    handles.pixhawk_battery_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in pixhawk_raw_imu.
function pixhawk_raw_imu_Callback(hObject, eventdata, handles)
% hObject    handle to pixhawk_raw_imu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pixhawk_raw_imu

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.pixhawk_raw_imu_plot = 1;
else
    handles.pixhawk_raw_imu_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in localization_sub.
function localization_sub_Callback(hObject, eventdata, handles)
% hObject    handle to localization_sub (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of localization_sub

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.localization_sub_plot = 1;
else
    handles.localization_sub_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)



% --- Executes on button press in px4flow_sub.
function px4flow_sub_Callback(hObject, eventdata, handles)
% hObject    handle to px4flow_sub (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of px4flow_sub

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.px4flow_sub_plot = 1;
else
    handles.px4flow_sub_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)


% --- Executes on button press in collision_avoidance.
function collision_avoidance_Callback(hObject, eventdata, handles)
% hObject    handle to collision_avoidance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of collision_avoidance

% Get status of check box
if (get(hObject,'Value') == get(hObject,'Max'))
    handles.collision_avoidance_plot = 1;
else
    handles.collision_avoidance_plot = 0;
end

% Update GUI Data
guidata(hObject, handles)



function test_date_Callback(hObject, eventdata, handles)
% hObject    handle to test_date (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of test_date as text
%        str2double(get(hObject,'String')) returns contents of test_date as a double

handles.date = get(hObject, 'String');

guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function test_date_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_date (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

date_temp = date;
date_format = datestr(date_temp, 'mm-dd-yyyy');
date_default = [date_format(1:2), '_', date_format(4:5), '_', date_format(7:10)];

set(hObject, 'String', date_default)
handles.date = get(hObject, 'String');

guidata(hObject, handles)


% --- Executes on button press in close_all.
function close_all_Callback(hObject, eventdata, handles)
% hObject    handle to close_all (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

close all


% --- Executes on button press in load_data.
function load_data_Callback(hObject, eventdata, handles)
% hObject    handle to load_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc

% Add file path
addpath(handles.date);

% File naming convention
filename = ['full_system_test', handles.num,'.txt'];

% Assign data to structure
handles.log = load_DVZ_log2(filename);



fprintf('Data loaded\n');

guidata(hObject, handles)



% --- Executes on button press in update.
function update_Callback(hObject, eventdata, handles)
% hObject    handle to update (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

close all



if handles.vicon_position_plot == 1;
    
    % Vicon Position
    plot_sub3(handles.log.time, handles.log.vicon_x, handles.log.vicon_y, ...
        handles.log.vicon_z, 'Time [s]', 'Vicon x [m]', 'Vicon y [m]', 'Vicon z [m]', ...
        'Vicon Position', 1);
    
end


if handles.pixhawk_attitude_plot == 1;
    
    % Pixhawk Euler Angles
    plot_sub3(handles.log.time, handles.log.pixhawk_roll, handles.log.pixhawk_pitch, ...
        handles.log.pixhawk_yaw, 'Time [s]', 'Pix $\phi$ [rad]', 'Pix $\theta$ [rad]', 'Pix $\psi$ [rad]', ...
        'Pixhawk Euler Angles', 1);
    
    % Pixhawk Euler Rates
    plot_sub3(handles.log.time, handles.log.pixhawk_rollspeed, handles.log.pixhawk_pitchspeed, ...
        handles.log.pixhawk_yawspeed, 'Time [s]', 'Pix $\dot{\phi}$ [rad/s]', 'Pix $\dot{\theta}$ [rad/s]', 'Pix $\dot{\Psi}$ [rad/s]', ...
        'Pixhawk Euler Rates', 1);
    
end


if handles.amcl_pose_plot == 1;
    
	amcl_yaw_wrapped = wrapToPi(handles.log.amcl_yaw);
	
    % AMCL Pose
    plot_sub3(handles.log.time, handles.log.amcl_x, handles.log.amcl_y, ...
        amcl_yaw_wrapped, 'Time [s]', 'AMCL x [m]', 'AMCL y [m]', 'AMCL $\Psi$ [rad]', ...
        'AMCL Pose', 1);
    
    % AMCL Position (Bird's Eye View)
    plot_single(handles.log.amcl_x, handles.log.amcl_y, 'X map [m]', 'Y map [m]', 'AMCL XY Position', 1)
    
    
end


if handles.altitude_controller_plot == 1;
    
    % Altitude Controller PID
    plot_3single(handles.log.time, handles.log.alt_RC_P, handles.log.alt_RC_I, ...
        handles.log.alt_RC_D, 'Time [s]','PWM','Altitude Controller PID',...
        'P', 'I', 'D',...
        1);
    
    % Altitude Data
    plot_sub3(handles.log.time, handles.log.unfiltered_px4flow_alt, handles.log.filtered_px4flow_alt, ...
        handles.log.local_zmap, 'Time [s]', 'Unfilt PX4 [m]', 'Filt PX4 [m]', 'Local Z [m]', ...
        'Altitude Data', 1);
    
end


if handles.velocity_controller_plot == 1;
    
    % Velocity X Actual and Target in Body
    plot_2single(handles.log.time, handles.log.velx_body, handles.log.target_velx_body,...
        'Time [s]','Velocity [m/s]','X Velocity Actual and Target in Body',...
        'Actual','Target',1);
    
    % Velocity Controller X PID
    plot_3single(handles.log.time, handles.log.velx_P, handles.log.velx_I, ...
        handles.log.velx_D, 'Time [s]','PWM','Velocity Controller X PID',...
        'P', 'I', 'D',...
        1);
    
    % Velocity Y Actual and Target in Body
    plot_2single(handles.log.time, handles.log.vely_body, handles.log.target_vely_body,...
        'Time [s]','Velocity [m/s]','Y Velocity Actual and Target in Body',...
        'Actual','Target',1);
    
    % Velocity Controller Y PID
    plot_3single(handles.log.time, handles.log.vely_P, handles.log.vely_I, ...
        handles.log.vely_D, 'Time [s]','PWM','Velocity Controller Y PID',...
        'P', 'I', 'D',...
        1);
    % Pitch and Roll Angles Sent
    plot_sub2(handles.log.time, handles.log.pitch_angle_send,handles.log.roll_angle_send,...
        'Time [s]','Pitch Angle [rad]','Roll Angle [rad]','Pitch and Roll Angles sent',1);
    
end


if handles.yaw_controller_plot == 1;
    
end


if handles.position_controller_plot == 1;
    
    % Position X From AMCL, Vicon and Target
    plot_3single(handles.log.time,handles.log.local_xmap,handles.log.vicon_x,...
        handles.log.target_x,'Time [s]','Position X [m]','Position X','AMCL',...
        'Vicon','Target',1);
    % Position X PID
    plot_3single(handles.log.time, handles.log.posx_P, handles.log.posx_I, ...
        handles.log.posx_D, 'Time [s]','Body Velocity [m/s]','Position Controller X PID',...
        'P', 'I', 'D',...
        1);
    % Position Y From AMCL, VICON and Target
    plot_3single(handles.log.time,handles.log.local_ymap,handles.log.vicon_y,...
        handles.log.target_y,'Time [s]','Position Y [m]','Position Y','AMCL',...
        'Vicon','Target',1);
    % Position Y PID
    plot_3single(handles.log.time, handles.log.posy_P, handles.log.posy_I, ...
        handles.log.posy_D, 'Time [s]','Body Velocity [m/s]','Position Controller Y PID',...
        'P', 'I', 'D',1);
end


if handles.rc_handset_pwm_plot == 1;
    % Handset Roll, Pitch, Yaw
    plot_sub3(handles.log.time,handles.log.handset_roll,handles.log.handset_pitch,...
        handles.log.handset_yaw,'Time [s]','Roll [PWM]','Pitch [PWM]','Yaw [PWM]',...
        'Handset Roll, Pitch, and Yaw',1)
    % Handset Throttle, Ch5, Ch6
    plot_sub3(handles.log.time,handles.log.handset_throttle,handles.log.handset_ch5,...
        handles.log.handset_ch6,'Time [s]','Throttle [PWM]','Channel 5 [PWM]','Channel 6 [PWM]',...
        'Handset Throttle, Channel 5, and Channel 6',1)
end


if handles.pixhawk_battery_plot == 1;
    % Battery Information
    plot_sub3(handles.log.time,handles.log.voltage/1000,handles.log.current/100,...
        handles.log.battery_remaining,'Time [s]','Voltage [volts]','Current [amps]',...
        'Battery Remaining [$\%$]','Battery Status',1)
end


if handles.pixhawk_raw_imu_plot == 1;
	
	% Convert accelerations to m/s^2 from milli-g's
	acc_scale = 9.81/1000;
	
	% Convert rotation rates to rad/s from mill-rad/s
	gyro_scale = 1/1000;
	
	% Convert acclerations to correct frame
	handles.log.pixhawk_xacc = -handles.log.pixhawk_xacc;
	handles.log.pixhawk_yacc = -handles.log.pixhawk_yacc;
	handles.log.pixhawk_zacc = -handles.log.pixhawk_zacc;
	
	% smoothed acc measurements
	accx_smooth = smooth(handles.log.pixhawk_xacc*acc_scale, 10);
	accy_smooth = smooth(handles.log.pixhawk_yacc*acc_scale, 10);
	accz_smooth = smooth(handles.log.pixhawk_zacc*acc_scale, 10);
	
	
    % Pixhawk Acceleration
    plot_sub3(handles.log.time,handles.log.pixhawk_xacc*acc_scale,handles.log.pixhawk_yacc*acc_scale,...
        handles.log.pixhawk_zacc*acc_scale,'Time [s]','Xb Acc [m/s$^{2}$]','Yb Acc [m/s$^{2}$]',...
        'Zb Acc [m/s$^{2}$]','Pixhawk Acceleration',1);
	
	% Smoothed pixhawk accleration
	plot_sub3(handles.log.time, accx_smooth, accy_smooth, accz_smooth, 'Time [s]','Xb Acc [m/s$^{2}$]','Yb Acc [m/s$^{2}$]',...
        'Zb Acc [m/s$^{2}$]','Pixhawk Acceleration - Smooth',1)
	
    % Pixhawk Gyro
    plot_sub3(handles.log.time,handles.log.pixhawk_xgyro*gyro_scale,handles.log.pixhawk_ygyro*gyro_scale,...
        handles.log.pixhawk_zgyro*gyro_scale,'Time [s]','X Gyro [rad/s]','Y Gyro [rad/s]',...
        'Z Gyro [rad/s]','Pixhawk Gyro',1);
    % Pixhawk Magnetometer
    plot_sub3(handles.log.time,handles.log.pixhawk_xmag,handles.log.pixhawk_ymag,...
        handles.log.pixhawk_zmag,'Time [s]','X Mag [mT]','Y Mag [mT]',...
        'Z Mag [mT]','Pixhawk Magnetometer',1);
end


if handles.localization_sub_plot == 1;
	
	
	% Differentiate kalman filter velocities
	dt = mean(diff(handles.log.time));
	kalman_accx_diff = smooth(diff(handles.log.local_velx_body)/dt, 10);
	kalman_accy_diff = smooth(diff(handles.log.local_vely_body)/dt, 10);
	kalman_accz_diff = smooth(diff(handles.log.local_velz_map)/dt, 10);

	
    % AMCL Position
    plot_sub3(handles.log.time,handles.log.local_xmap,handles.log.local_ymap,...
        handles.log.local_zmap,'Time [s]','X Map [m]','Y Map [m]',...
        'Z Map [m]','AMCL Position',1);
    % AMCL Velocity
    plot_sub3(handles.log.time,handles.log.local_velx_body,handles.log.local_vely_body,...
        handles.log.local_velz_map,'Time [s]','Xb Vel [m/s]','Yb Vel [m/s]',...
        'Zm Vel [m/s]','Kalman Filter Velocities',1);
	
	% AMCL Acceleration (Velocity diff)
    plot_sub3(handles.log.time(1:end-1),kalman_accx_diff,kalman_accy_diff,...
        kalman_accz_diff,'Time [s]','Xb acc [m/$s^{2}$]','Yb acc [m/$s^{2}$]',...
        'Zm Vel [m/$s^{2}$]','Kalman Filter Acceleration - Diff of Velocity',1);
	
	
    % AMCL Acceleration
    plot_sub3(handles.log.time,handles.log.local_accx_body,handles.log.local_accy_body,...
        handles.log.local_accz_map,'Time [s]','Xb Acc [m/$s^{2}$]','Yb Acc [m/$s^{2}$]',...
        'Zm Acc [m/$s^{2}$]','Kalman Filter Accelerations',1);
    % AMCL Psi
    plot_single(handles.log.time,handles.log.local_psi_map,'Time [s]',...
        '$\psi$ Map [rad]','AMCL $\psi$',1);
    
end


if handles.px4flow_sub_plot == 1;
    % Flow Altitude and Quality
    plot_sub2(handles.log.time,handles.log.px4flow_alt,handles.log.px4flow_quality,...
        'Time [s]','Altitude [m]','Quality','PX4 Flow Altitude and Quality',1);
end


if handles.collision_avoidance_plot == 1;
    figure
    Coll_Avoid_flag = handles.log.hokuyo_min_range < handles.log.hokuyo_min_dist;
    for i = 1:length(handles.log.time)
        if Coll_Avoid_flag == 1
            clf
            angles = linspace(-120,120,length(handles.log.hokuyo_ranges));
            X = handles.log.hokuyo_ranges(i,:) .* cosd(angles);
            Y = handles.log.hokuyo_ranges(i,:) .* sind(angles);
            scatter(X,Y,'filled'),hold all
            quiver(0,0,handles.log.hokuyo_target_velx_body(i),handles.log.hokuyo_target_vely_body(i))
            theta = handles.log.hokuyo_angle(i);
            r = handles.log.hokuyo_min_range(i);
            plot([0,0],[r*cos(theta),r*sin(theta)],'k--')
            grid on
            xlabel('X Position [m]','FontSize',18)
            ylabel('Y Position [m]','FontSize',18)
            h = title(['Collision Avoidance Scans at t = ',num2str(handles.log.time(i)),'sec']);
            set(h,'FontSize',20)
            legend('Laser Scans','Desired Velocity','Nearest Scan Range')
            pause(.01)
        end
    end
    
    
end


