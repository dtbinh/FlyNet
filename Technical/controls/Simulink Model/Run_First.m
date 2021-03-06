clear all
close all
clc
% The purpose of this script is to define vehicle properties such as Mass
% and Inertia as well as to define the vehicle configuration
% characteristics that will be used in the simulink model

%% Define physical properties associated with a multirotor vehicle ================================

g = 9.71;               % Gravitational Constant (m/s^2)
Mass = 2.4;             % Define Mass of Vehicle (kg)
Ixx = .05;              % Inertia of Vehicle about x-axis (kg/m^3)
Iyy = .05;              % Inertia of Vehicle about x-axis (kg/m^3)
Izz = .05;              % Inertia of Vehicle about x-axis (kg/m^3)

% 0<=delta<=1 where pwm = (1-2)-1 PulseWidthModulation

k1 = 0.127;             % Motor force constant F = k1*delta (N)   
k2 = 0.053;             % Motor torque constant tau = k2*delta (N)
l = .15;                % Distance from vehicle center to motor (m)

%% Performance Limits ==============================================================================

% == None

%% Multi-Rotor Vehicle Configuration (i.e. Quad, Hex, Octo) ========================================

% Forces & Torques generated by motors | Select Quad or Hex rotor set up

configNum = input('Please select which rotor configuration you will be flying: "1" = quad-rotor, "2" = hex-rotor');
configOrient = input('Please select which orientation you will be flying: "+" = 3 rotor forward & "x" = 4');

% Quadrotor=========================================================================================

if configNum == 1 && configOrient == 3
% (+) 1 forward, 2 side, one back rotor config. 
    MotorForceTorque = [k1   k1    k1  k1        
                        0    -l*k1 0   l*k1        
                        l*k1 -l*k1 0   0           
                        -k2  k2    -k2 k2	];     
% (x) 2 Forward, 2 back rotor config.
elseif configNum == 1 && configOrient == 4
    MotorForceTorque = [k1      k1      k1      k1  
                        -l*k1   -l*k1   l*k1    l*k1 
                        l*k1    -l*k1   -l*k1   l*k1 
                        k2     -k2      k2     -k2   ];
            

% Hexarotor=========================================================================================
% (x) 2 foward, 2 side, 2 back rotor config. 
elseif configNum == 2 && configOrient == 4
MotorForceTorque = [k1             k1    k1              k1              k1   k1
                    -.5*l*k1       -l*k1 -.5*l*k1        .5*l*k1         l*k1 .5*l*k1
                    sqrt(3)/2*l*k1 0     -sqrt(3)/2*l*k1 -sqrt(3)/2*l*k1 0    sqrt(3)/2*l*k1
                    -k2            k2    -k2             k2              -k2  k2            ];       

% (+) 1 foward, 4 side, 1 back rotor config. 
elseif configNum == 2 && configOrient == 3
MotorForceTorque = [k1             k1              k1              k1    k1             k1
                    0              -sqrt(3)/2*k1*l -sqrt(3)/2*k1*l 0     sqrt(3)/2*k1*l sqrt(3)/2*k1*l
                    l*k1           l*k1/2          -l*k1/2         -l*k1 -l*k1/2        l*k1/2
                    -k2            k2              -k2             k2    -k2            k2            ];
end  
%===================================================================================================

%% Psuedo Inverse
if configNum == 1 && configOrient == 3
% (+) 1 forward, 2 side, one back rotor config. 
    MotorForceTorqueP =  [ 1/(4*k1) -1/(2*k1*l)  1/(k1*l)  1/(4*k2)
                           1/(4*k1) -1/(2*k1*l)         0  1/(4*k2)
                           1/(4*k1)  1/(2*k1*l) -1/(k1*l) -3/(4*k2)
                           1/(4*k1)  1/(2*k1*l)         0  1/(4*k2)];
% (x) 2 Forward, 2 back rotor config.
elseif configNum == 1 && configOrient == 4
    MotorForceTorqueP =  [ 1/(4*k1) -1/(4*k1*l)  1/(4*k1*l)  1/(4*k2)
                           1/(4*k1) -1/(4*k1*l) -1/(4*k1*l) -1/(4*k2)
                           1/(4*k1)  1/(4*k1*l) -1/(4*k1*l)  1/(4*k2)
                           1/(4*k1)  1/(4*k1*l)  1/(4*k1*l) -1/(4*k2)];


% Hexarotor=========================================================================================
% (x) 2 foward, 2 side, 2 back rotor config. 
elseif configNum == 2 && configOrient == 4
MotorForceTorqueP = [ 1/(6*k1) -1/(6*k1*l)  3^(1/2)/(6*k1*l) -1/(6*k2)
                      1/(6*k1) -1/(3*k1*l)                 0  1/(6*k2)
                      1/(6*k1) -1/(6*k1*l) -3^(1/2)/(6*k1*l) -1/(6*k2)
                      1/(6*k1)  1/(6*k1*l) -3^(1/2)/(6*k1*l)  1/(6*k2)
                      1/(6*k1)  1/(3*k1*l)                 0 -1/(6*k2)
                      1/(6*k1)  1/(6*k1*l)  3^(1/2)/(6*k1*l)  1/(6*k2)];    

% (+) 1 foward, 4 side, 1 back rotor config. 
elseif configNum == 2 && configOrient == 3
MotorForceTorqueP = [ 1/(6*k1) -1/(6*k1*l)  3^(1/2)/(6*k1*l) -1/(6*k2)
                      1/(6*k1) -1/(3*k1*l)                 0  1/(6*k2)
                      1/(6*k1) -1/(6*k1*l) -3^(1/2)/(6*k1*l) -1/(6*k2)
                      1/(6*k1)  1/(6*k1*l) -3^(1/2)/(6*k1*l)  1/(6*k2)
                      1/(6*k1)  1/(3*k1*l)                 0 -1/(6*k2)
                      1/(6*k1)  1/(6*k1*l)  3^(1/2)/(6*k1*l)  1/(6*k2)];
end  

%% Outputs
multiParam = [0 Mass Ixx Iyy Izz k1 k2 l g];   % 0 is place holder time vector
configParam = [0 configNum configOrient];

% zero = [0;0;0;0;0;0];
% F_T_Matrix.time=[0 0 0 0 0 0];
% F_T_Matrix.signal.values = MotorForceTorque;
F_T_Matrix = MotorForceTorqueP;










