function controls = controller(pose,ref,type)
% Pose and ref are both given in global coordinates

if type == 1 % Position reference
    controls = position_controller(pose,ref);
elseif type == 2 % Velocity reference
    controls = velocity_controller(pose,ref);
end

end
function controls = position_controller(pose,ref)
Kpx = 40;
Kix = 0.001;
Kdx = -2;
Kpy = 40;
Kiy = 0.001;
Kdy = -2;
Kpz = 20;
Kiz = 0.001;
Kdz = -2;
Kpyaw = 1;
Kiyaw = 0;
Kdyaw = 0;

persistent eix
persistent eiy
persistent eiz
persistent eiyaw

if isempty(eix)
    eix = 0;
    eiy = 0;
    eiz = 0;
    eiyaw = 0;
end

ex = ref(1) - pose(1);
eix = eix + ex;
edx = pose(7);

ey = ref(2) - pose(2);
eiy = eiy + ey;
edy = pose(8);

ez = ref(3) - pose(3);
eiz = eiz + ez;
edz = pose(9);

eyaw = ref(4) - pose(6);
eiyaw = eiyaw + eyaw;
edyaw = pose(12);

vx = Kpx*ex + Kix*eix + Kdx*edx;
vy = Kpy*ey + Kiy*eiy + Kdy*edy;
vz = Kpz*ez + Kiz*eiz + Kdz*edz;
yawrate = Kpyaw*eyaw + Kiyaw*eiyaw + Kdyaw*edyaw;

vxmax = 2;
vymax = 2;
vzmax = 2;

if abs(vx) > vxmax
    vx = vxmax*vx/abs(vx);
end

if abs(vy) > vymax
    vy = vymax*vy/abs(vy);
end

if abs(vz) > vzmax
    vz = vzmax*vz/abs(vz);
end

vel = [vx,vy,vz,yawrate]';

controls = velocity_controller(pose,vel);

end
function controls = velocity_controller(pose,ref)
yawrate = ref(4);
Kproll = 200;
Kiroll = 0.;
Kdroll = -5;
Kppitch = 200;
Kipitch = 0.;
Kdpitch = -5;
Kpthrust = 10;
Kithrust = 0.0;
Kdthrust = -2;

persistent eiroll
persistent eipitch
persistent eithrust

if isempty(eiroll)
    eiroll = 0;
    eipitch = 0;
    eithrust = 0;
end

roll = pose(4);
pitch = pose(5);
yaw = pose(6);

Rbodytovehicle = [1 0 0;0 cos(roll) sin(roll);0 -sin(roll) cos(roll)]*...
    [cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch)]*...
    [cos(yaw) sin(yaw) 0;-sin(yaw) cos(yaw) 0;0 0 1];

ref_body = Rbodytovehicle'*ref(1:3);

epitch = ref_body(1) - pose(4);
eroll = ref_body(2) - pose(5);
ethrust = ref_body(3) - pose(6);
eiroll = eiroll+eroll;
eipitch = eipitch+epitch;
eithrust = eithrust + ethrust;
edroll = pose(8);
edpitch = pose(7);
edthrust = pose(9);

roll = Kproll*eroll+Kiroll*eiroll+Kdroll*edroll;
pitch = Kppitch*epitch+Kipitch*eipitch+Kdpitch*edpitch;
thrust = Kpthrust*ethrust+Kithrust*eithrust+Kdthrust*edthrust;

rollmax = 15*pi/180;
pitchmax = 15*pi/180;
thrustmax = 40;
yawratemax = 30*pi/180;

if abs(roll) > rollmax
    roll = rollmax*roll/abs(roll);
end
if abs(pitch) > pitchmax
    pitch = pitchmax*pitch/abs(pitch);
end
if thrust > thrustmax
    thrust = thrustmax;
end
if abs(yawrate) > yawratemax
    yawrate = yawratemax*yawrate/abs(yawrate);
end

controls = [roll;-pitch;yawrate;20/(cos(pose(4))*cos(pose(5))) + thrust];

end
