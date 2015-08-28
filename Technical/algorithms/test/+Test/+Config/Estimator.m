classdef Estimator < handle
    properties (Constant)
        Q = diag([.05,.05]);
        R = diag([.01,.01]);
    end
end