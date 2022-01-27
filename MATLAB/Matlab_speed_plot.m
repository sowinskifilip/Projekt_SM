close all; clear all;

%% Serial port set up

if ~exist('huart', 'var')
    huart = serial('COM3','BaudRate',9600,'Terminator','LF', 'Timeout', 10);
    fopen(huart);
end

%% Characteristic plot

N = 1000000;  % number of data points
h = figure();
    pSpeed = plot(nan(N,1),nan(N,1), 'b');
    hold on;
    pSpeed_ref = plot(nan(N,1),nan(N,1), 'r');
    xlabel('Time [s]');
    ylabel('Speed [rpm]');
    legend('Motor Speed', 'Reference Speed');
    hold on; grid on;

%% Perform experiment 
    
k = 1;
t = 0;
ts = 0.1;

while(1)
    data = jsonencode(fscanf(huart))
    data_reg = regexp(data, '\-?\d{3}\.\d{3}', 'match')
    if ~isempty(data_reg(1))
        speed = str2double(data_reg(1));
        ref_speed = str2double(data_reg(2));
        updateplot(pSpeed, speed, t, k, N);
        updateplot(pSpeed_ref, ref_speed, t, k, N);
        drawnow;
    end
    
    t = t + ts;
    k = k + 1;
    
    if ~isempty(h.CurrentCharacter)
        break;
    end
end

%% Close serial port and remove handler    
fclose(huart);
delete(huart);
clearvars('huart');

%% Plot update function: save only last N samples
function updateplot(hplot, y, x, k, N)
    if k > N
        hplot.YData = circshift(hplot.YData, -1); 
        hplot.YData(end) = y; 
        hplot.XData = circshift(hplot.XData, -1); 
        hplot.XData(end) = x; 
    else
        hplot.YData(k) = y; 
        hplot.XData(k) = x; 
    end
end