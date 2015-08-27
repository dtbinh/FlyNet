dt = .1;
numFrames = 1000;
poseData = Test.Utils.generate_pose_data('square',dt,numFrames);


gainStruct = struct();
gainStruct.Kp_position = 1;
gainStruct.Ki_position = 1;
gainStruct.Kd_position = 1;

controller = Data.Controller.ControllerExample(gainStruct);

% Your goal is create your own version of the file ControllerExample that
% actually calculates desired velocities (the current one that I made is
% broken since the calc_desired_velocities function does nothing). Then I
% want you to plot the desired velocities as a function of time in this
% function using the data in the variable poseData, which contains x-y
% positions over time. Start by copying the ControllerExample code into a
% new script and saving it as ControllerLastName, i.e. Drew's would be
% ControllerEllison. Then edit anything you see fit to get the controller
% to work.