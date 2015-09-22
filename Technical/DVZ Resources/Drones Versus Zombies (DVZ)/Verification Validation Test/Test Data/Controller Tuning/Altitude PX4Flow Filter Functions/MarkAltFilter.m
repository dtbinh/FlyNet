function [filtered_alt] = MarkAltFilter(time,alt)

for i = 1:length(alt)
    filtered_alt(i) = alt(i);
    if i > 1
        if alt(i) == 0
            filtered_alt(i) = alt(i-1);
            if alt(i) == 0 && alt(i-1) == 0
                filtered_alt(i) = filtered_alt(i-1);
            end
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