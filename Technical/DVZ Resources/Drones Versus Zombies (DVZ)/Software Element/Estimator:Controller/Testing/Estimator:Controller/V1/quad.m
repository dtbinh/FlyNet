classdef quad < handle
    properties
        pose
    end
    methods
        function obj = quad(pose)
            obj.pose = pose;
        end
        function update_pose(obj,controls)
            roll = controls(1);
            pitch = controls(2);
            yawrate = controls(3);
            thrust = controls(4);
            global dt
            
            roll = obj.pose(4);
            pitch = obj.pose(5);
            yaw = obj.pose(6);
            
            Rbodytovehicle = [1 0 0;0 cos(roll) sin(roll);0 -sin(roll) cos(roll)]*...
                [cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch)]*...
                [cos(yaw) sin(yaw) 0;-sin(yaw) cos(yaw) 0;0 0 1];
            
            if obj.pose(7) ~= 0
                xdd = [0;0;-20] + Rbodytovehicle*[0;0;thrust] - ...
                    .5*2*[obj.pose(7);obj.pose(8);obj.pose(9)].^2 .*...
                    ([obj.pose(7);obj.pose(8);obj.pose(9)]./...
                    abs([obj.pose(7);obj.pose(8);obj.pose(9)]));
            else
                xdd = [0;0;-20] + Rbodytovehicle*[0;0;thrust];
            end
            obj.pose(1:3) = obj.pose(1:3) +obj.pose(7:9)*dt+.5*dt^2*xdd;
            obj.pose(7:9) = obj.pose(7:9) + xdd*dt;
            obj.pose(6) = obj.pose(6) + yawrate*dt;
            obj.pose(4) = controls(1);
            obj.pose(5) = controls(2);
        end
    end
end