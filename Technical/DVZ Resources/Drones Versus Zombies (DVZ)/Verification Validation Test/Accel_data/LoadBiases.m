function [bias_data] = LoadBiases(filename)
raw_data = load(filename);

bias_data.time = raw_data(:,1)-raw_data(1,1);
bias_data.accel_x_bias = raw_data(:,2);
bias_data.accel_y_bias = raw_data(:,3);
bias_data.accel_z_bias = raw_data(:,4);

bias_data.ax = raw_data(:,5);
bias_data.ay = raw_data(:,6);
bias_data.az = raw_data(:,7);

end

