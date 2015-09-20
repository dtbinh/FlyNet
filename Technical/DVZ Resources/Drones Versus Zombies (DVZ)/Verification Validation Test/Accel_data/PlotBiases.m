function [] = PlotBiases(bias_data)

figure
subplot(3,1,1)
plot(bias_data.time,bias_data.ax)
ylabel('ax value')
subplot(3,1,2)
plot(bias_data.time,bias_data.accel_x_bias)
ylabel('Accel X Bias')

actual_estimate = bias_data.ax-bias_data.accel_x_bias;
subplot(3,1,3)
plot(bias_data.time,actual_estimate)
ylabel('X Kalman Output')
%%
figure
subplot(3,1,1)
plot(bias_data.time,bias_data.ay)
ylabel('ay value')
subplot(3,1,2)
plot(bias_data.time,bias_data.accel_y_bias)
ylabel('Accel y Bias')

actual_estimate = bias_data.ay-bias_data.accel_y_bias;
subplot(3,1,3)
plot(bias_data.time,actual_estimate)
ylabel('Y Kalman Output')
%%
figure
subplot(3,1,1)
plot(bias_data.time,bias_data.az)
ylabel('ay value')
subplot(3,1,2)
plot(bias_data.time,bias_data.accel_z_bias)
ylabel('Accel z Bias')

actual_estimate = bias_data.az-bias_data.accel_z_bias;
subplot(3,1,3)
plot(bias_data.time,actual_estimate)
ylabel('Z Kalman Output')

end

