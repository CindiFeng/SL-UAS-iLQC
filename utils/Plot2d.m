function util_plot2d(wp,t_interval,xdata,ydata)
% wp: array containing waypoint 
% goal: goal position 

plot(xdata,ydata,'b-','LineWidth',1);  
hold on;

% plot waypoints
if size(wp,1) > 1
    for i = size(wp,1)-1
        plot(t_interval*i,wp(i),'r+','LineWidth',1.5);
        text(t_interval*i,wp(i),'Waypoint',...
             'VerticalAlignment','top','HorizontalAlignment','left'); 
    end
end

% plot goal line
yline(wp(end),'--','Goal','LineWidth',0.8);