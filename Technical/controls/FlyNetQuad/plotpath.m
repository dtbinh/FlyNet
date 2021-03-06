profile on

% cmdx = squeeze(path.x.Data);
% cmdy = squeeze(path.y.Data);
% cmdz = squeeze(path.z.Data);
cmdx   = path.X_cmd.Data;
cmdy   = path.Y_cmd.Data;
cmdz   = path.Z_cmd.Data;
cmdpsi = path.Psi_cmd.Data;

actx = posN.Data;
acty = posE.Data;
actz = posU.Data;
actpsi = attPsi.Data;

% figure(2)
% hold on
% grid on
% plot3(cmdx, cmdy, cmdz, 'b', 'linewidth', 2);
% plot3(actx, acty, actz, 'r--', 'linewidth', 2);
% legend('Commanded Path', 'Predicted Path', 'Location', 'Southeast')
% xlabel('North [in]'); ylabel('East [in]'); zlabel('Up [in]');

figure(3)
grid on
% hold on; plot(posN.Time, actx); plot(path.X_cmd.Time, cmdx);
% hold on; plot(posE.Time, acty); plot(path.Y_cmd.Time, cmdy);
% hold on; plot(posU.Time, actz); plot(path.Z_cmd.Time, cmdz);
hold on; plot(attPsi.Time, actpsi); plot(path.Psi_cmd.Time, cmdpsi);
xlabel('Time [s]'); ylabel('Angle [rad]');

% profile viewer