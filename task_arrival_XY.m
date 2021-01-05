function taskXYs = task_arrival_XY(numTask, probArrival, xSize, ySize, endID)
b = ceil(endID/xSize);
taskXYs = [];
for i=1:numTask
    if probArrival >= rand 
        taskXYs = [taskXYs;randi(xSize) randi([ySize-b ySize])];
    end
end