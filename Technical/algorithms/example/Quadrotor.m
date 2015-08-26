classdef Quadrotor < handle
    properties (Access = private)
        type
        currentTime
        numMotors
        motorType
        motors = {};
        % All Quadrotor variable will be in inertial coordinate system
        xPosition;
        yPosition;
        zPosition;
        xVelocity;
        yVelocity;
        zVelocity;
        
    end
    methods
        function [obj] = Quadrotor()
            obj.type = DVZ.Common.Config.Quadrotor.type;
            obj.motorType = DVZ.Common.Config.Quadrotor.motorType;
            switch obj.motorType
                case DVZ.Common.Globals.MOTOR_SIM
                    obj.motors = cell(4,1);
                    for motorIdx = 1:obj.numMotors
                        obj.motors{motorIdx} = DVZ.QuadSim.MotorSim();
                    end
            end
        end
    end
end