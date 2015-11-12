cmdx = squeeze(path.x.Data);
cmdy = squeeze(path.y.Data);
cmdz = squeeze(path.z.Data);

actx = posN.Data;
acty = posE.Data;
actz = posU.Data;

figure(1)
hold on
plot3(cmdx, cmdy, cmdz, 'b');
plot3(actx, acty, actz, 'r--');