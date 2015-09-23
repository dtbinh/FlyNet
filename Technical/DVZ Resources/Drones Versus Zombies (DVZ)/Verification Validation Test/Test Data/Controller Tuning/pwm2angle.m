function [angle] = pwm2angle(pwm,radio_trim,radio_max,radio_min)
dead_zone = 0;
angle_high = 4500;
if pwm >= (radio_trim + dead_zone)
    centiangle = angle_high*(pwm - (radio_trim + dead_zone))/(radio_max - (radio_trim + dead_zone));
elseif pwm < (radio_trim - dead_zone)
    centiangle = angle_high*(pwm - (radio_trim - dead_zone))/((radio_trim - dead_zone) - radio_min);
else
    centiangle = 0;
end
angle = (centiangle)/100;
% angle_max = 45;
% angle_min = -45;
% 
% if angle >= angle_max
%     angle = angle_max;
% elseif angle < angle_min
%     angle = angle_min;
% end

end