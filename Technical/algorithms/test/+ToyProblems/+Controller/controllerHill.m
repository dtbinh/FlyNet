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
            dt = .1;
            velocityDes = struct();
            %%
            % Calculate error
            preErrorX = (desiredPosition.x - position.x(1));            
            preErrorY = (desiredPosition.y - position.y(1));
            errorX = (desiredPosition.x - position.x(2));            
            errorY = (desiredPosition.y - position.y(2));
            % Calculate derivative
            derX = (errorX-preErrorX)/dt;
            derY = (errorY-preErrorY)/dt;
            
                      
            %%
            velocityDes.x = obj.Kp_position*errorX + obj.Kd_position*derX;
            velocityDes.y = obj.Kp_position*errorY + obj.Kd_position*derY;
        end
    end 
end