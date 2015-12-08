clear all
close all
clc

startPercentage = .30;
endPercentage = .6;

directoryName = '4Dec_unpacked/2015-12-04-19-58-40';
fileNames = dir(directoryName);
fileNames = {fileNames(3:end).name};

data = struct();

for fileIdx = 1:numel(fileNames)
    fieldName = strrep(fileNames{fileIdx}(1:end-4),'-','_');
    data.(fieldName) = readtable(fullfile(directoryName,fileNames{fileIdx}));
end

setpointTime = data.mavros_setpoint_position_local.Header_secs- min(data.mavros_setpoint_position_local.Header_secs) + ...
    data.mavros_setpoint_position_local.Header_nsecs * 1e-9;
poseTime = data.mavros_local_position_local.Header_secs - min(data.mavros_local_position_local.Header_secs) + ...
    data.mavros_local_position_local.Header_nsecs * 1e-9;

xsetpoint = data.mavros_setpoint_position_local.Pose_x;
ysetpoint = data.mavros_setpoint_position_local.Pose_y;
zsetpoint = data.mavros_setpoint_position_local.Pose_z;
yawsetpoint = 2*acos(data.mavros_setpoint_position_local.Orientation_w)*180/pi;

xpose = data.mavros_local_position_local.Pose_x;
ypose = data.mavros_local_position_local.Pose_y;
zpose = data.mavros_local_position_local.Pose_z;
yawpose = 2*acos(data.mavros_local_position_local.Orientation_w)*180/pi;


startIdxPose = floor(numel(poseTime) * startPercentage) + 1;
endIdxPose = floor(numel(poseTime) * endPercentage);
startIdxSetpoint = floor(numel(setpointTime) * startPercentage) + 1;
endIdxSetpoint = floor(numel(setpointTime) * endPercentage);

figure()
plot(poseTime(startIdxPose:endIdxPose),xpose(startIdxPose:endIdxPose)), hold on
plot(setpointTime(startIdxSetpoint:endIdxSetpoint),xsetpoint(startIdxSetpoint:endIdxSetpoint))
xlabel('Time (s)','FontSize',16)
ylabel('X Position (m)','FontSize',16)
grid on

figure()
plot(poseTime(startIdxPose:endIdxPose),ypose(startIdxPose:endIdxPose)), hold on
plot(setpointTime(startIdxSetpoint:endIdxSetpoint),ysetpoint(startIdxSetpoint:endIdxSetpoint))
xlabel('Time (s)','FontSize',16)
ylabel('Y Position (m)','FontSize',16)
grid on

figure()
plot(poseTime(startIdxPose:endIdxPose),zpose(startIdxPose:endIdxPose)), hold on
plot(setpointTime(startIdxSetpoint:endIdxSetpoint),zsetpoint(startIdxSetpoint:endIdxSetpoint))
xlabel('Time (s)','FontSize',16)
ylabel('Z Position (m)','FontSize',16)
grid on

figure
plot(xpose(startIdxPose:endIdxPose),ypose(startIdxPose:endIdxPose)), hold on
plot(xsetpoint(startIdxSetpoint:endIdxSetpoint),ysetpoint(startIdxSetpoint:endIdxSetpoint))
xlabel('X Position (m)','FontSize',16)
ylabel('Y Position (m)','FontSize',16)
grid on

figure
plot(poseTime(startIdxPose:endIdxPose),yawpose(startIdxPose:endIdxPose)), hold on
plot(setpointTime(startIdxSetpoint:endIdxSetpoint),yawsetpoint(startIdxSetpoint:endIdxSetpoint))
xlabel('Time (s)','FontSize',16)
ylabel('Heading (deg)','FontSize',16)
grid on
