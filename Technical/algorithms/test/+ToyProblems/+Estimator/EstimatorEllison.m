classdef EstimatorEllison < handle
    properties
        state % [x,y,vx,vy]'
        covariance
        F
        G
        H
        R
        Q
        time
    end
    methods
        function [obj] = EstimatorEllison()
            obj.state = [0,0,0,0]';
            obj.covariance = diag([100,100,100,100]);
            obj.R = Test.Config.Estimator.R;
            obj.Q = Test.Config.Estimator.Q;
            obj.time = -.1;
        end
        
        function [velocityEstimate] = calc_estimated_velocity(obj,position)
            velocityEstimate = struct();
            prev_x = obj.state(1);
            prev_y = obj.state(2);
            cur_x = position.x;
            cur_y = position.y;
            dt = position.time - obj.time;
            
            obj.state(1) = position.x;
            obj.state(2) = position.y;
            unfilt_vx = (cur_x - prev_x)/dt;
            unfilt_vy = (cur_y - prev_y)/dt;
            
            obj.state(3) = obj.state(3) * .95 + unfilt_vx * .05 ;
            obj.state(4) = obj.state(4) * .95 + unfilt_vy * .05;
            obj.time = position.time;
            velocityEstimate.x = obj.state(3);
            velocityEstimate.y = obj.state(4);
        end
    end 
end