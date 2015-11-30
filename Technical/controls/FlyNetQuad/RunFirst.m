clc; clear;

load('SupportData\IC.mat');
load('SupportData\pathFlemming.mat');
load('SupportData\quadModel_X.mat');

% load('Tests\IC_tests.mat');
% load('Tests\Path_x_response.mat');
% load('SupportData\quadModel_X.mat');

profile on

model = 'MainModel_v2.slx';
load_system(model)
sim(model)

% cmdx = path.X_cmd.Data;
% cmdy = path.Y_cmd.Data;
% cmdz = path.Z_cmd.Data;
cmdx = squeeze(path.x.Data);
cmdy = squeeze(path.y.Data);
cmdz = squeeze(path.z.Data);

actx = posN.Data;
acty = posE.Data;
actz = posU.Data;

figure(2)
hold on
grid on
plot3(cmdx, cmdy, cmdz, 'b', 'linewidth', 2);
plot3(actx, acty, actz, 'r--', 'linewidth', 2);
legend('Commanded Path', 'Predicted Path', 'Location', 'Southeast')
xlabel('North [in]'); ylabel('East [in]'); zlabel('Up [in]');

% profile viewer