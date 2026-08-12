function updateTrackPlot(t1,t2)
%
%*This function needs to be called with (nan,nan) to be initialised*

global figures;
global data;

track = data.track;
tf = figures.track;
ax = figures.track.ax;

if isnan(t1) %If there is no cursor, simply plot the track (initialisation)
    %-Initialise axes
    cla(ax);
    hold on
    axis equal
    %-Create plot objects for the track and path
    tf.centerline = plot(ax,track.x,track.y,'--k');
    tf.path = plot(ax,track.xopt,track.yopt,'LineWidth',1.5,'Color',[0.25 0.5 1]);
    tf.limits = plot(ax, track.Xl(:,1),track.Xl(:,2),'k', track.Xr(:,1),track.Xr(:,2),'k');
    %-Create objects for the cursor position with visibility off
    tf.pos1 = plot(ax, track.x(1), track.y(1), 'o', 'Visible', 'off', 'Color', [0.4 0.8 0.1], 'LineWidth',5,'MarkerSize',5);
    tf.pos2 = plot(ax, track.x(1), track.y(1), 'o', 'Visible', 'off', 'Color', [1 0.4 0.2], 'LineWidth',5,'MarkerSize',5);
elseif isnan(t2) %If there is only one cursor
    if t1 >= track.s(1) && t1 <= track.s(end) %If the value is within the limits of the simulation
        %-Update cursor 1
        idx1 = find(abs(data.s_full-t1)==min(abs(data.s_full-t1)),1);
        set(tf.pos1, 'Xdata', track.xopt(idx1), 'Ydata', track.yopt(idx1),'Visible', 'on'); %Update cursor on track plot
    end
else
    if t1 >= track.s(1) && t2 <= track.s(end)
        %-Update cursor 1
        idx1 = find(abs(data.s_full-t1)==min(abs(data.s_full-t1)),1);
        set(tf.pos1, 'Xdata', track.xopt(idx1), 'Ydata', track.yopt(idx1),'Visible', 'on'); %Update cursor on track plot
        %-Update cursor 2
        idx2 = find(abs(data.s_full-t2)==min(abs(data.s_full-t2)),1);
        set(tf.pos2, 'Xdata', track.xopt(idx2), 'Ydata', track.yopt(idx2),'Visible', 'on'); %Update cursor on track plot
    end
end

figures.track = tf; %Update track container

end