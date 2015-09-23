function [t,target_psi,target_x,target_y] = defineWaypoints(wp)
switch wp
    case 'step'
        %%%%% Define Step %%%%%
        t = [0:0.1:30]';
        for i = 1:length(t)
            if t(i) <= 15
                target_psi.signals.values(i,1) = 0;
                target_x.signals.values(i,1) = 0;
                target_y.signals.values(i,1) = 0;
            elseif t(i) > 15
                target_psi.signals.values(i,1) = 0;
                target_x.signals.values(i,1) = 1.5;
                target_y.signals.values(i,1) = 1.5;
            end
        end
        target_psi.time = t;
        target_x.time = t;
        target_y.time = t;
        target_psi.signals.dimensions = 1;
        target_x.signals.dimensions = 1;
        target_y.signals.dimensions = 1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'hourglass'
        %%%%% Define Hourglass %%%%%
        t = [0:0.1:110]';
        for i = 1:length(t)
            if t(i) <= 15
                target_psi.signals.values(i,1) = pi/2;
                target_x.signals.values(i,1) = 0;
                target_y.signals.values(i,1) = 0;
            elseif t(i) > 15 && t(i) <= 35
                target_psi.signals.values(i,1) = pi/2;
                target_x.signals.values(i,1) = -1.5;
                target_y.signals.values(i,1) = -1.5;
            elseif t(i) > 35 && t(i) <= 55
                target_psi.signals.values(i,1) = pi/2;
                target_x.signals.values(i,1) = 1.5;
                target_y.signals.values(i,1) = 1.5;
            elseif t(i) > 55 && t(i) <= 75
                target_psi.signals.values(i,1) = pi/2;
                target_x.signals.values(i,1) = 1.5;
                target_y.signals.values(i,1) = -1.5;
            elseif t(i) > 75 && t(i) <= 95
                target_psi.signals.values(i,1) = pi/2;
                target_x.signals.values(i,1) = -1.5;
                target_y.signals.values(i,1) = 1.5;
%             elseif t(i) > 47 && t(i) <= 90
%                 target_psi.signals.values(i,1) = 3*pi/2;
%                 target_x.signals.values(i,1) = -1.5;
%                 target_y.signals.values(i,1) = -1.5;
            elseif t(i) > 95
                target_psi.signals.values(i,1) = pi/2;
                target_x.signals.values(i,1) = -1.5;
                target_y.signals.values(i,1) = -1.5;
            end
        end
        target_psi.time = t;
        target_x.time = t;
        target_y.time = t;
        target_psi.signals.dimensions = 1;
        target_x.signals.dimensions = 1;
        target_y.signals.dimensions = 1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'figure8'
        %%%%% Define Figure 8 %%%%%
        t = [0:0.1:157]';
        scalex = 1.5;
        scaley = 3;
        scalet = 0.04;
        x = scalex*sin(scalet*t);
        y = scaley*sin(2*scalet*t)/2;
        
        target_psi.time = t;
        target_psi.signals.values = zeros(length(t),1);
        target_psi.signals.dimensions = 1;
        target_x.time = t;
        target_x.signals.values = x;
        target_x.signals.dimensions = 1;
        target_y.time = t;
        target_y.signals.values = y;
        target_y.signals.dimensions = 1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'square'
        %%%%% Define Square %%%%%
        t = [0:0.1:105]';
        for i = 1:length(t)
            if t(i) <= 15
                target_psi.signals.values(i,1) = 0;
                target_x.signals.values(i,1) = 0;
                target_y.signals.values(i,1) = 0;
            elseif t(i) > 15 && t(i) <= 30
                target_psi.signals.values(i,1) = pi/2;
                target_x.signals.values(i,1) = -1.5;
                target_y.signals.values(i,1) = -1.5;
            elseif t(i) > 30 && t(i) <= 45
                target_psi.signals.values(i,1) = pi;
                target_x.signals.values(i,1) = -1.5;
                target_y.signals.values(i,1) = 1.5;
            elseif t(i) > 45 && t(i) <= 60
                target_psi.signals.values(i,1) = 3*pi/2;
                target_x.signals.values(i,1) = 1.5;
                target_y.signals.values(i,1) = 1.5;
            elseif t(i) > 60 && t(i) <= 75
                target_psi.signals.values(i,1) = 2*pi;
                target_x.signals.values(i,1) = 1.5;
                target_y.signals.values(i,1) = -1.5;
            elseif t(i) > 75
                target_psi.signals.values(i,1) = 0;
                target_x.signals.values(i,1) = -1.5;
                target_y.signals.values(i,1) = -1.5;
            end
        end
        target_psi.time = t;
        target_x.time = t;
        target_y.time = t;
        target_psi.signals.dimensions = 1;
        target_x.signals.dimensions = 1;
        target_y.signals.dimensions = 1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'circle'
        %%%%% Define Circle %%%%%
        t = [0:0.1:160]';
        scalet = 0.1;
        scale_psi = 0.025;
        r = 2;
        x = r*sin(scalet*t);
        y = r*cos(scalet*t);
        psi = (2*pi)*cos(scale_psi*t);
        target_psi.time = t;
        target_psi.signals.values = psi;
        target_psi.signals.dimensions = 1;
        target_x.time = t;
        target_x.signals.values = x;
        target_x.signals.dimensions = 1;
        target_y.time = t;
        target_y.signals.values = y;
        target_y.signals.dimensions = 1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%% 
end
end