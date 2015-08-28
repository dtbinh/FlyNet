classdef ControllerExample < handle
    properties
        Kp_position
        Ki_position
        Kd_position
    end
    methods
        function [obj] = ControllerExample()
            obj.Kp_position = Test.Config.Controller.Kp_position;
            obj.Ki_position = Test.Config.Controller.Ki_position;
            obj.Kd_position = Test.Config.Controller.Kd_position;
        end
        
        function [velocityDes] = calc_desired_velocity(obj,position,desiredPosition)
            velocityDes = struct();
            velocityDes.x = 0;
            velocityDes.y = 0;
        end
    end 
end