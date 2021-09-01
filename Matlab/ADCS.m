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
BT_device_ID = 'btspp://98D341FD55DE';

% Init Bluetooth connection with Device
BT = Bluetooth(BT_device_ID, 1); % Connect to device

% Inint Stream Handler
fopen(BT);
flushinput(BT);
% Send The Estabilsh Reading Signal
fwrite(BT, 1, 'int8');


while ~stop
    % The BlueTooth will send 2 float values each time
    % represented in bytes
    % size can be 2 instead of [1,2]
    % and you still can access it the same way
    data        = fread(BT, [1, 7], 'float');
    timing      = data(1);
    yaw         = data(4);
    speedZ      = data(7);
    % Plot Data
    %addpoints(data_line,...
    %    datenum( timing ), speedZ*3.14/180*60 );
    addpoints(data_line,...
        datenum( timing ), yaw );
    % Update axes
    ax.XLim = datenum([timing-15 timing]);
    % Represent in time format and apply keeplimits 
    % for preserving the axis limits
    %datetick('x', 'keeplimits');
    drawnow;
end



