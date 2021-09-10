clc, clearvars, close all % Initialize Application
delete(instrfind); % reset all devices connections

% Initialize Variables
% Implementing RealTime Ploting
i           = 0;
stop        = false;        % control infinit loop
data_line   = animatedline; % animated line that has no data and adds it to the current axes
ax          = gca;          % Use to get and set properties of the current axes

% Scan For BlueTooth Device 
% use BTdevices.RemoteNames to Get BT Devices Names
% and then get the corresponding RemoteID
%BTdevices = instrhwinfo('bluetooth');
%BT_device_ID = 'btspp://98D371FD5D12';
BT_device_ID = 'btspp://98D341FD55DE';


% Init Bluetooth connection with Device
BT = Bluetooth(BT_device_ID, 1); % Connect to device

% Inint Stream Handler
fopen(BT);
flushinput(BT);
flushoutput(BT);
% Send The Estabilsh Reading Signal
%fwrite(BT, 1, 'int8');
t_old = 0 ;
i = 1 ;

i_old = 1 ;
while ~stop
    % The BlueTooth will send 2 float values each time
    % represented in bytes
    % size can be 2 instead of [1,2]
    % and you still can access it the same way
    fwrite(BT, 1, 'int8');
    data = fread(BT, [1, 3], 'float');
    
    if(length(data) <= 0)
        fprintf('No data availabe !');
        pause(2);
        continue;
    end
    
    timing      = data(1);
    yaw         = data(2);
    speedZ      = data(3);
    % Plot Data
    
    dataLog.t(i) = timing ;
    dataLog.yaw(i) = yaw ;
    dataLog.speedZ(i) = speedZ ;
    
    if i == 1
        ax1 = subplot(2,1,1) ;
        line1 = plot(dataLog.t,dataLog.yaw) ;
        hold on
        grid on
        ax2 = subplot(2,1,2) ;
        line2 = plot(dataLog.t,dataLog.speedZ) ;
        hold on
        grid on
        line1.XDataSource = 'dataLog.t' ;
        line2.XDataSource = 'dataLog.t' ;
        line1.YDataSource = 'dataLog.yaw' ;
        line2.YDataSource = 'dataLog.speedZ' ;
        
    else
        if i-i_old>5
            if timing>20
                xlim(ax1,[timing-20,timing])
                xlim(ax2,[timing-20,timing])
            end
            refreshdata(line1);
            refreshdata(line2);
            drawnow
           i_old = i ; 
        end
    end
    i = i + 1 ;
%     drawnow;
    fprintf('time = %f, dt = %f, yaw = %f, omegaZ = %f\n',timing,timing-t_old,yaw,speedZ)
    t_old = timing;
end



