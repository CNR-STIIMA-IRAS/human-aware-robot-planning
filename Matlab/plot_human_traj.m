figure; 
for iFrame = 1:kinect_data.xvalues.signals.dimensions
    hold on;
    plot3( kinect_data.xvalues.signals.values(:,iFrame), kinect_data.yvalues.signals.values(:,iFrame), kinect_data.zvalues.signals.values(:,iFrame), 'LineWidth', 2 );
    xlabel('X[m]')
    ylabel('Y[m]')
    zlabel('Z[m]')
    grid on;
    axis equal;
end  


cmap = hsv(size(robottrajs,2));  %# Creates a 6-by-3 set of colors from the HSV colormap
for iTraj = 1:size(robottrajs,2)    
    hold on;
    plot3( robottrajs{iTraj}(:,1), robottrajs{iTraj}(:,2), robottrajs{iTraj}(:,3), '*','Color',cmap(iTraj,:) ) %# Plot each column with a different color
end

hold on;
plot3( kinect_data.og_points(1:1:length(kinect_data.og_points),1), kinect_data.og_points(1:1:length(kinect_data.og_points),2), kinect_data.og_points(1:1:length(kinect_data.og_points),3),'.' )
%plot3( kinect_data.og_points(:,1), kinect_data.og_points(:,2), kinect_data.og_points(:,3),'.' )
%plot3(startpoint(1),startpoint(2),startpoint(3),'Db');
%hold on;
%robot.plot(startjoint);
