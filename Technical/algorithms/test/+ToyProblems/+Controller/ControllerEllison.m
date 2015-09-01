classdef ControllerEllison < handle
    properties
        Kp_position
        Ki_position
        Kd_position
    end
    methods
        function [obj] = ControllerEllison()
            obj.Kp_position = Test.Config.Controller.Kp_position;
            obj.Ki_position = Test.Config.Controller.Ki_position;
            obj.Kd_position = Test.Config.Controller.Kd_position;
        end
        
        function [velocityDes] = calc_desired_velocity(obj,position,desiredPosition)
            velocityDes = struct();
            % Calculate the position error
            error = struct();
            error.x = desiredPosition.x - position.x;
            error.y = desiredPosition.y - position.y;
            velocityDes.x = error.x * obj.Kp_position;
            velocityDes.y = error.y * obj.Kp_position;
        end
    end 
end