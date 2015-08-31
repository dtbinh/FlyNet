dt = .1;
numFrames = 1000;
noiseStruct = struct();
noiseStruct.position = .01;
poseData = Test.Utils.generate_pose_meas_data('square',dt,numFrames,noiseStruct);

estimator = ToyProblems.Estimator.EstimatorExample();

% Your goal is to create your own version of the file EstimatorExample
% that actually calculates estimated velocities (the current one that I made
% is broken since the calc_estimated_velocities function does nothing). Then
% I want you to plot the estimated velocities as a function of time in this
% function using the data in the variable poseData, which contains x-y
% position measurements over time (the code for this already exists below). Start by
% copying the EstimatorExample code into a new script and saving it as
% EstimatorLastName, i.e. Drew's would be EstimatorEllison. Then edit
% anything you see fit to get the estimator to work.

estimatedVelocity = struct();
estimatedVelocity.x = [];
estimatedVelocity.y = [];
for frameIdx = 1:numel(poseData.time)
    pose = struct();
    pose.x = poseData.x(frameIdx);
    pose.y = poseData.y(frameIdx);
    pose.time = poseData.time(frameIdx);
    velEst = estimator.calc_estimated_velocity(pose);
    estimatedVelocity.x(frameIdx) = velEst.x;
    estimatedVelocity.y(frameIdx) = velEst.y;
end

plot(poseData.time,estimatedVelocity.x),hold on
plot(poseData.time,estimatedVelocity.y)
grid on
xlabel('Time (s)','FontSize',12)
ylabel('Estimated Velocity (m/s)','FontSize',12)
legend('X velocity','Y velocity')
