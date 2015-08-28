function [poseMeas] = generate_pose_meas_data(pattern,dt,numFrames,noiseStruct)

poseTruth = Test.Utils.generate_pose_data(pattern,dt,numFrames);

poseMeas = struct();
poseMeas.x = poseTruth.x + randn(size(poseTruth.x))*noiseStruct.position;
poseMeas.y = poseTruth.y + randn(size(poseTruth.y))*noiseStruct.position;
poseMeas.time = poseTruth.time;
end