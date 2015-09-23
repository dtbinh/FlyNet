function [filtered_alt] = EddyAltFilter(time,alt)

time_drop = 0;
vel_drop = 0;
last_meas = 0;
for i = 1:length(alt)
    if i == 1
        filtered_alt(i) = alt(i);
        vel_standard = 0;
    else
        delta_t = time(i) - time(i-1);
        if alt(i) == 0
            time_drop = time_drop + delta_t;
            if alt(i) == 0 && alt(i-1) == 0
                filtered_alt(i) = filtered_alt(i-1) + (vel_standard*delta_t)*(delta_t/time_drop);
            else
                filtered_alt(i) = alt(i-1) + (vel_standard*delta_t);
            end    
            delta_pos = filtered_alt(i) - filtered_alt(i-1);
        else
            filtered_alt(i) = alt(i);
            delta_pos = filtered_alt(i) - filtered_alt(i-1);
            if time_drop ~= 0
                if last_meas ~= 0
                    vel_drop = (alt(i) - last_meas)/time_drop;
                end
            end
            if vel_drop == 0
                vel_standard = delta_pos/delta_t;
            else
                vel_standard = vel_drop;
                vel_drop = 0;
            end
            last_meas = alt(i);
            filtered_alt(i) = alt(i);
            time_drop = 0;
        end
    end
end

figure
hold on
grid on
for j = 1:length(alt)
    plot(time(j),filtered_alt(j),'bo')
    plot(time(j),alt(j),'rx')
%     waitforbuttonpress()
    pause(0.01)
end
xlabel('Time [s]')
ylabel('Alt [m]')

end