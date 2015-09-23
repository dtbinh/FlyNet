function [] = plotWaypointPath(t,x_desired_out,y_desired_out,x_out,y_out,psi_desired_out,psi_out,wp)
switch wp
    case 'step'
        figure
        hold on
        grid on
        plot(x_desired_out.signals.values,y_desired_out.signals.values,'b')
        plot(x_out.signals.values,y_out.signals.values,'r')
        for i = 1:length(t)
            p1 = plot(x_desired_out.signals.values(i),y_desired_out.signals.values(i),'go','MarkerFaceColor','g');
            p2 = plot(x_out.signals.values(i),y_out.signals.values(i),'k^','MarkerFaceColor','k');
            pose = [x_out.signals.values(i); y_out.signals.values(i); psi_out.signals.values(i)];
            [h1,h2] = plot_heading(pose,1);
            pause(0.01)
            xlabel('X [m]')
            ylabel('Y [m]')
            if i ~= length(t)
                delete(p1)
                delete(p2)
                delete(h1)
                delete(h2)
            end
        end
        legend([p1 p2],'Target','Quad','Location','Best')
    case 'hourglass'
        c1_center = [0, 0];
        r1 = 0.1;
        c2_center = [-1.5, -1.5];
        r2 = 0.1;
        c3_center = [-1.5, 1.5];
        r3 = 0.1;
        c4_center = [1.5, -1.5];
        r4 = 0.1;
        c5_center = [1.5, 1.5];
        r5 = 0.1;
        
        writeobj = VideoWriter('hourglass.avi');
        open(writeobj);
        
        figure
        hold on
        grid on
        plot(x_desired_out.signals.values,y_desired_out.signals.values,'b')
        plot(x_out.signals.values,y_out.signals.values,'r')
        viscircles(c1_center,r1,'EdgeColor','k');
        viscircles(c2_center,r2,'EdgeColor','k');
        viscircles(c3_center,r3,'EdgeColor','k');
        viscircles(c4_center,r4,'EdgeColor','k');
        viscircles(c5_center,r5,'EdgeColor','k');
        set(gca,'XDir','reverse')
        for i = 1:length(t)
            p1 = plot(x_desired_out.signals.values(i),y_desired_out.signals.values(i),'go','MarkerFaceColor','g');
            p2 = plot(x_out.signals.values(i),y_out.signals.values(i),'k^','MarkerFaceColor','k');
            pose = [x_out.signals.values(i); y_out.signals.values(i); psi_out.signals.values(i)];
            [h1,h2] = plot_heading(pose,1);
            M(i) = getframe;
            pause(0.01)
            xlabel('X [m]')
            ylabel('Y [m]')
            if i ~= length(t)
                M(i) = getframe;
                delete(p1)
                delete(p2)
                delete(h1)
                delete(h2)
            end
        end
        movie2avi(M,'hourglass.avi','compression','none')
        legend([p1 p2],'Target','Quad','Location','Best')
    case 'figure8'
        figure
        hold on
        grid on
        plot(x_desired_out.signals.values,y_desired_out.signals.values,'b')
        plot(x_out.signals.values,y_out.signals.values,'r')
        for i = 1:length(t)
            p1 = plot(x_desired_out.signals.values(i),y_desired_out.signals.values(i),'go','MarkerFaceColor','g');
            p2 = plot(x_out.signals.values(i),y_out.signals.values(i),'k^','MarkerFaceColor','k');
            pose = [x_out.signals.values(i); y_out.signals.values(i); psi_out.signals.values(i)];
            [h1,h2] = plot_heading(pose,1);
            pause(0.01)
            xlabel('X [m]')
            ylabel('Y [m]')
            if i ~= length(t)
                delete(p1)
                delete(p2)
                delete(h1)
                delete(h2)
            end
        end
        legend([p1 p2],'Target','Quad','Location','Best')
    case 'square'
        c1_center = [0, 0];
        r1 = 0.1;
        c2_center = [-1.5, -1.5];
        r2 = 0.1;
        c3_center = [-1.5, 1.5];
        r3 = 0.1;
        c4_center = [1.5, -1.5];
        r4 = 0.1;
        c5_center = [1.5, 1.5];
        r5 = 0.1;
        
        figure
        hold on
        grid on
        plot(x_desired_out.signals.values,y_desired_out.signals.values,'b')
        plot(x_out.signals.values,y_out.signals.values,'r')
        viscircles(c1_center,r1,'EdgeColor','k');
        viscircles(c2_center,r2,'EdgeColor','k');
        viscircles(c3_center,r3,'EdgeColor','k');
        viscircles(c4_center,r4,'EdgeColor','k');
        viscircles(c5_center,r5,'EdgeColor','k');
        for i = 1:length(t)
            p1 = plot(x_desired_out.signals.values(i),y_desired_out.signals.values(i),'go','MarkerFaceColor','g');
            p2 = plot(x_out.signals.values(i),y_out.signals.values(i),'k^','MarkerFaceColor','k');
            pose = [x_out.signals.values(i); y_out.signals.values(i); psi_out.signals.values(i)];
            [h1,h2] = plot_heading(pose,1);
            pause(0.01)
            xlabel('X [m]')
            ylabel('Y [m]')
            if i ~= length(t)
                delete(p1)
                delete(p2)
                delete(h1)
                delete(h2)
            end
        end
        legend([p1 p2],'Target','Quad','Location','Best')
        case 'circle'
        figure
        hold on
        grid on
        axis equal
        plot(x_desired_out.signals.values,y_desired_out.signals.values,'b')
        plot(x_out.signals.values,y_out.signals.values,'r')
        for i = 1:length(t)
            p1 = plot(x_desired_out.signals.values(i),y_desired_out.signals.values(i),'go','MarkerFaceColor','g');
            p2 = plot(x_out.signals.values(i),y_out.signals.values(i),'k^','MarkerFaceColor','k');
            pose = [x_out.signals.values(i); y_out.signals.values(i); psi_out.signals.values(i)];
            [h1,h2] = plot_heading(pose,1);
            pause(0.01)
            xlabel('X [m]')
            ylabel('Y [m]')
            if i ~= length(t)
                delete(p1)
                delete(p2)
                delete(h1)
                delete(h2)
            end
        end
        legend([p1 p2],'Target','Quad','Location','Best')
end
end