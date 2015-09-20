classdef vicon_filter < handle
    properties
        x_0
        x
        P
        Q
        R
    end
    methods
        function obj = vicon_filter(Q,R)
            obj.Q = Q;
            obj.R = R;
            obj.x_0 = [];
            obj.P = 100*eye(15);
            obj.x = [];
        end
        function estimate = propogate(obj,pose)
            global dt
            if isempty(obj.x)
                obj.x = [pose];
            else
                F = eye(15);
                F(1,7) = dt;
                F(2,8) = dt;
                F(3,9) = dt;
                F(4,10) = dt;
                F(5,11) = dt;
                F(6,12) = dt;
                F(1,13) = dt^2/2;
                F(2,14) = dt^2/2;
                F(3,15) = dt^2/2;
                F(7,13) = dt;
                F(8,14) = dt;
                F(9,15) = dt;
                obj.x = F*obj.x;
                obj.P = F*obj.P*F' + obj.Q;
            end
            estimate = obj.x;
        end
        function estimate = update(obj,pose)
            H = zeros(6,15);
            H(1,1) = 1;
            H(2,2) = 1;
            H(3,3) = 1;
            H(4,4) = 1;
            H(5,5) = 1;
            H(6,6) = 1;
            z = pose(1:6) - H*obj.x;
            S = H*obj.P*H' + obj.R;
            K = obj.P*H'*inv(S);
            
            obj.x = obj.x + K*z;
            obj.P = (eye(15) - K*H)*obj.P;
            
            estimate = obj.x;
        end
    end
end