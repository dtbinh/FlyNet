function [pwm] = angle2pwm(angle,radio_trim_pos,radio_max,radio_min)
angle_high = 4500;
centiangle = (angle*100);
if centiangle > 0 
   pwm = ((centiangle*(radio_max - radio_trim_pos)))/angle_high + radio_trim;
else
   pwm = ((centiangle*(radio_trim-radio_min)) )/angle_high+ radio_trim; 
end

end