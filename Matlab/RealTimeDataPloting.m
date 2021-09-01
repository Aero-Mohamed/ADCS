clc, clearvars, close all % Initialize Application

% After Testing the feature of 'drawnow' i found out
% that it consomes high computeional power from CPU(40%) & GPU(10%)
% An alternative solution is provide below :)

% Init Vars
stop        = false;        % stop flag
i           = 1;            % Iterator if needed
%data_line   = animatedline; % animated line that has no data and adds it to the current axes
xLimit      = 15; % seconds
ax          = gca;          % Use to get and set properties of the current axes
x1          = [];
y1          = [];
plot1       = plot(ax, 0, 0, 'XDataSource', 'x1', 'YDataSource', 'y1');


while ~stop
    % Generate Data to plot
    t =  datetime('now');
    
    % append data
    x1 = [x1 datenum(t)];
    y1 = [y1 rand(1)*100];
    %addpoints(data_line,datenum(t),rand(1)*100)
    
    % Update axes
    ax.XLim = datenum([t-seconds(xLimit) t]);
    
    % Represent in time format and apply keeplimits 
    % for preserving the axis limits
    datetick('x', 'keeplimits'); 
    
    refreshdata;
    drawnow;
    %pause(0.2); % add point every 200ms
end
