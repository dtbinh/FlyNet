classdef EstimatorExample < handle
    properties
        state % [x,y,vx,vy]'
        covariance
        F
        G
        H
        R
        Q
    end
    methods
        function [obj] = EstimatorExample()
            obj.state = [0,0,0,0]';
            obj.covariance = diag([100,100,100,100]);
            obj.R = Test.Config.Estimator.R;
            obj.Q = Test.Config.Estimator.Q;
        end
        
        function [velocityEstimate] = calc_estimated_velocity(obj,position)
            velocityEstimate = struct();
            velocityEstimate.x = 0;
            velocityEstimate.y = 0;
        end
    end 
end