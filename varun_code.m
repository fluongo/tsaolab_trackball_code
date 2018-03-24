% initialize
m = MotionSensor('/dev/ttyACM1');
m.StartReading;

%%
m.StartReading;

currX = zeros(3000, 1); 
currY = zeros(3000, 1);

nn = 1;
while nn<2000
    if mod(nn, 3000)== 0
        nn =1;
    end
    pause(0.02)
    [x, y] = m.ReadSensor;
    currX(nn) = x;currY(nn) = y;
    hold off
    plot([1:nn], currX(1:nn), 'r');hold on; 
    plot([1:nn], currY(1:nn), 'k');
    
    nn = nn+1;
end

m.StopReading