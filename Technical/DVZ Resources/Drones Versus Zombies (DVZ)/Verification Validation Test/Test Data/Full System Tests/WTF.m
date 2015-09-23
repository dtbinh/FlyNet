x = -pi:pi/20:pi;
y1 = sin(x);
y2 = cos(x);

figure
plot(x,y1,'-ro',x,y2,'-.b')
legend('sin(x)','cos(x)','Location','northwest')