% tells Arduino to have motor go to pos = 1 rad 1 second after hitting
% start.  Plots position on graphs



% port must be set to the communication port used by the Arduino
% you can find the port by going to 'tools -> Port' in the Arduion
% application. For a PC, it will be something like COM6, and for a Mac, it
% will be something like /dev/cu.usbmodem1451
%port='COM6';
port='COM48';
obj = serial(port, 'BaudRate', 115200);
obj.terminator = char(10);
fopen(obj)
%
% do a read to get Ready! from Arduino
%
dummy = fgets(obj);
%
% Read and display some data
%
for i=1:5,
    data = fgets(obj);
    disp(data)
end;
%
% Read data after sending command to Arduino
%
disp('Starting Counting Event in Arduino')
fprintf(obj,'%s\n','S'); % send start signal to Arduino
data=[];
k=0

% Read Data from Arduino
data = fgets(obj);
% Display what you got
disp(data)
while (~strncmp(data,'Finished',8)) % Until Arduino signals that it is done
    k=k+1;
    % change string data to cell array using tab delimiter
    dataarray = strsplit(data,char(9));
    % save data converting strings to numbers
    Time(k) = eval(dataarray{1});
    Count(k) = eval(dataarray{2});
    % Read Data from Arduino
    data = fgets(obj);
    % Display what you got
    disp(data)
end;
fclose(obj)

%
% Plot results
%
plot(Time,Count)
xlabel('Time (ms)')
ylabel('Position (rad)')
xlim([0,1950]);

%
% Save results in a .mat file to use later
%
save somedata.mat Time Count
    

    