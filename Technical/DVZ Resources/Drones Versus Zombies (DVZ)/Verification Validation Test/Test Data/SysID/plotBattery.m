function plotBattery(batt)

figure
subplot(311)
plot(batt.time,batt.voltage,'r'),grid
title('Battery Info')
ylabel('Voltage [V]')
subplot(312)
plot(batt.time,batt.current,'r'),grid
ylabel('Current [A]')
subplot(313)
plot(batt.time,batt.batt_remaining,'r'),grid
xlabel('Time [s]')
ylabel('%')

end