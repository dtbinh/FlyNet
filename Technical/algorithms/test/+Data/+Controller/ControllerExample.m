classdef ControllerExample < handle
    properties
        Kp_position
        Ki_position
        Kd_position
    end
    methods
        function [obj] = ControllerExample(gainStruct)
            obj.Kp_position = gainStruct.Kp_position;
            obj.Ki_position = gainStruct.Ki_position;
            obj.Kd_position = gainStruct.Kd_position;
        end
        
        function [velocityDes] = calc_desired_velocity(obj,position)
            velocityDes = struct();
            velocityDes.x = 0;
            velocityDes.y = 0;
        end
    end 
end