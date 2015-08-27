function [pose] = generate_pose_data(pattern,dt,numFrames)

% Setup structure for pose data
pose = struct();

time = [0:dt:dt*(numFrames-1)]';
switch pattern
    case 'square'
        % Set corner positions
        corners = [0,0;10,0;10,10;0,10;0,0];
        positions = [];
        % Loop over corner positions, generating frame by frame data
        for cornerIdx = 1:size(corners,1)-1
            positions = [positions;linspace(corners(cornerIdx,1),corners(cornerIdx+1,1),numFrames/4)',...
                linspace(corners(cornerIdx,2),corners(cornerIdx+1,2),numFrames/4)']; 
        end
        pose.time = time;
        pose.x = positions(:,1);
        pose.y = positions(:,2);
end

end