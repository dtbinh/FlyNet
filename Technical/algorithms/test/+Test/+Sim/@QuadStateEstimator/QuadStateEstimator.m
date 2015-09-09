classdef QuadStateEstimator < handle
    properties
        x % State
        P % Covariance
        IMU % IMU class for measurement generation
        Q % Process Noise
        R % Measurement Noise
    end
    methods
        function [obj] = QuadStateEstimator()
            
        end
        
        function time_propagation(obj)
            
        end
        
        function measurement_update(obj)
            
        end
    end
end