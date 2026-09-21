function [t, data] = serial_reader_nV2(numDataPoints)
% Copyright 2014 The MathWorks, Inc.
% with some modificaitons by Eric Mehiel

%% Create serial object for Arduino
if (~isempty(serialportfind))
    fclose(serialportfind); % close all ports to start, just to be sure...
    delete(serialportfind);
end

s = serialport('/dev/cu.usbmodem1101', 115200);  % change the COM Port number as needed

%% Connect the serial port to Arduino

try
    fopen(s);
catch err
    fclose(serialportfind);
    delete(serialportfind);
    error('Make sure you select the correct COM Port where the Arduino is connected.');
end

%% Read  the data from Arduino

i = 0; % counter
data = zeros(numDataPoints,5);
t = zeros(numDataPoints,1);

tempData = fscanf(s); % Clear the serial buffer

timer = tic; % Start timer
while i < numDataPoints
    i = i + 1;
    % Read buffer data
    data(i,:) = fscanf(s, "%f"); % Change format string as needed
    disp("Reading Serial Data..." + i);
    % Read time stamp
    t(i) = toc(timer);
end
fclose(s);
delete(s);
clear s;