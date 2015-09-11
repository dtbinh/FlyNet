dt = .1;
numFrames = 1000;
poseData = Test.Utils.generate_pose_data('square',dt,numFrames);

controller = ToyProblems.Controller.ControllerExample();

% Your goal is to create your own version of the file ControllerExample
% that actually calculates desired velocities (the current one that I made
% is broken since the calc_desired_velocities function does nothing). Then
% I want you to plot the desired velocities as a function of time in this
% function using the data in the variable poseData, which contains x-y
% positions over time (the code for this already exists below). Start by
% copying the ControllerExample code into a new script and saving it as
% ControllerLastName, i.e. Drew's would be ControllerEllison. Then edit
% anything you see fit to get the controller to work. Once you get it to
% work, feel free to tune the controller.

desiredVelocity = struct();
desiredVelocity.x = [];
desiredVelocity.y = [];
desiredPosition.x = 0;
desiredPosition.y = 0;
for frameIdx = 1:(numel(poseData.time)-1)
    pose = struct();
    pose.x = [poseData.x(frameIdx) poseData.x(frameIdx+1)];
    pose.y = [poseData.y(frameIdx) poseData.y(frameIdx+1)];
    pose.time = poseData.time(frameIdx);
    velDesCalc = controller.calc_desired_velocity(pose,desiredPosition);
    desiredVelocity.x(frameIdx) = velDesCalc.x;
    desiredVelocity.y(frameIdx) = velDesCalc.y;

end
%% Correct vector size to match frameIdx 
desiredVelocity.x = [desiredVelocity.x desiredVelocity.x(frameIdx)];
desiredVelocity.y = [desiredVelocity.y desiredVelocity.y(frameIdx)];
%%
plot(poseData.time,desiredVelocity.x),hold on
plot(poseData.time,desiredVelocity.y)
grid on
xlabel('Time (s)','FontSize',12)
ylabel('Desired Velocity (m/s)','FontSize',12)
legend('X velocity','Y velocity')
