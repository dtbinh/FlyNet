classdef QuadrotorUKF < handle
    properties
        % Attitude
        phi
        theta
        psi
        
        % Attitude Rates/accelerations
        phid 
        phidd 
        thetad
        thetadd
        yawd
        yawdd
        
        % Body velocities
        velx_body
        vely_body
        velz_body
        
        % Body level velocities
        velx_level
        vely_level
        velz_level
        
        % Inertial Positions
        x_inertial
        y_inertial
        z_inertial
        T % Thrust
        
        % Moments of inertia
        Jx 
        Jy
        Jz
        
        % IMU Class
        IMU
    end
    methods
        function [obj] = QuadrotorUKF()
            
        end
        function prop_time(obj)
            
        end
    end
end