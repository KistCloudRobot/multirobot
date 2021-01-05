function taskXYs = taskID2XY(taskIDs, itemEndID, xSize, ySize)
taskXYs = [];
if isempty(taskIDs)
    error('taskID2XY: empty taskID');
end
    
for i=1:length(taskIDs)
    if  taskIDs(i) > 0
        [X, Y] = ind2sub([xSize, ySize], xSize*ySize - (itemEndID - taskIDs(i)));
        taskXYs = [taskXYs; X Y];
    end
end
