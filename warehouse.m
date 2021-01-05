%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Changjoo Nam (cjnam@kist.re.kr)

% This script runs coordination of multiple robots in a warehouse setup.
% Pick up tasks arrive in an online fashion following a Poisson distribution.
% The robots pick the items from shelves and return the item to a station located at the bottom of the warehouse.

% This script is developed with MATLAB R2018b version. May not properly work with other versions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
clear;close all
showFig = 2;
nRobot = 3;
numTaskPerStep = 10;
probArrival = 1;
alpha = 2;
xSize = 20;
ySize = 20;
itemEndID = 200; %xSize*ySize > itemEndID + ySize*2
tMax = 1000;
tStep = 0;
bSize = 10;
taskNum = 0;
timeRobotMove = 0;
timeRobotWait = 0;
numAssignedTasks = 0;
numBundledTasks = 0;
timeCompletedTasks = 0;
timeInQueueTasks = 0;
timeInBundle = 0;
timeInExecution = 0;
numCollison = 0;
gridMap = zeros(xSize,ySize);
robotX = randi(xSize, [nRobot, 1]);
robotY = randperm(ySize, nRobot)'; 
robotXY = [robotX, robotY];
robotSet = [(1:nRobot)' robotXY zeros([nRobot 1])];
for i=1:nRobot
    robotBundles{i} = [];
    robotBundlesRecord{i} = [];
    robotPaths{i} = [];
end
stationXY = [ceil(xSize/2), 1];
rackXY = taskID2XY(1:itemEndID, itemEndID, xSize, ySize);
taskQueue = [];
taskAll = [];
collisonFrame = [];
hPath = zeros([1, nRobot]);

if showFig > 0
    hFig = figure; hold on;
    set(hFig, 'Position', [20, 70, xSize*35+30, ySize*35+30]);
    [X,Y]=ind2sub([xSize, ySize], 1:xSize*ySize);
    gcolor = [0.8 0.8 0.8];
    plot(X,Y,'color', gcolor, 'marker', 'square', 'linestyle', 'none', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor',gcolor, 'MarkerFaceColor',gcolor);
    axis off
    hRobot = plot(robotSet(:,2), robotSet(:,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    plot(rackXY(:,1), rackXY(:,2), 'bs', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    plot(stationXY(1), stationXY(2), 'ks', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')
end

while tStep < tMax
    %% Task arrival process
    % Choose a random number at a random time step
    % The number is drawn from a uniform distribution.
    % The timing is determined by a Poisson process.
    % Tasks are saved in the task waiting queue.
    %% Task arrival process
    if mod(tStep,alpha)==0      
        newTaskXYs = task_arrival_XY(numTaskPerStep, probArrival, xSize, ySize, itemEndID);
        taskIdx = 1:size(newTaskXYs,1);
        if ~isempty(newTaskXYs)
            taskXY = newTaskXYs;
            taskQueue = [taskQueue; tStep.*ones(length(taskIdx),1) (taskNum+taskIdx)' taskXY];
            taskAll = [taskAll; (taskNum+taskIdx)' tStep.*ones(length(taskIdx),1) -100.*ones(length(taskIdx),1)];
            taskNum = taskNum+length(taskIdx);
        end
    end

    %% Task allocation overview
    % Identify all available robots (m)
    % Choose x tasks from the queue where x is the optimal bundle size. x should be smaller than or equal to p, which is the capacity of the item-transporting tray.
    % Compute Manhattan distances from the robots to the tasks (m by x matrix)
    % Allocate the robots to the tasks
    %   Allocation methods (0-worst, 2-best)
    %   -1. Greedy (first come first served)
    %   0. Run an ILP based on the costs computed considering only the distance
    %   1. Plan paths collaboratively considering the conflicts among the robots included in the allocation
    %   2. Plan paths collaboratively considering the conflicts among the robots included in the allocation and other working robots
    %     Comment: 2 is not possible. 1 is the best one that I can do but 0 is
    %     still fine. But I need to admit that the allocation would not be optimal.
    %     1 still could be suboptimal since it does not consider dynamic
    %     obstacles when computing the allocation. 
    % Generate paths (A*)
    
    % (implementation) -1. Greedy allocation: first come first served
    idleRobotIdx = robotSet((robotSet(:,4)==0),1);%Identify available robots
    for i = 1:length(idleRobotIdx)
        thisRobotIdx = idleRobotIdx(i);
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotBundle = robotBundles{thisRobotIdx};
        timeInBundle = timeInBundle + size(thisRobotBundle,1);
        if ~isempty(taskQueue) && size(thisRobotBundle,1) < bSize            
            bFill = bSize - size(thisRobotBundle,1);
            if size(taskQueue,1) >= bFill
                thisRobotBundle = [thisRobotBundle; taskQueue(1:bFill,:)]; %bundling
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(1:bFill,1));
                numBundledTasks = numBundledTasks + bFill;
                taskQueue(1:bFill,:) = []; %removing from the queue                
            else
                thisRobotBundle = [thisRobotBundle; taskQueue]; %bundling
                timeInQueueTasks = timeInQueueTasks + sum(tStep-taskQueue(:,1));
                numBundledTasks = numBundledTasks + size(taskQueue,1);
                taskQueue = []; %removing from the queue
            end
            robotBundles{thisRobotIdx} = thisRobotBundle; %register bundle
        end
        if (size(thisRobotBundle,1) >= bSize)
            numAssignedTasks = numAssignedTasks + bSize;
            robotBundlesRecord{thisRobotIdx} = thisRobotBundle; %register bundle
            robotSet(thisRobotIdx,4) = 2;%mark this robot working for vending
            % A* path planning
            startXY = thisRobotXY;
            goalXY  = thisRobotBundle(1, 3:4);
            thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
            modWhen = -1;
            robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
            if showFig == 2
                hTask(thisRobotBundle(1,2)) = plot(thisRobotBundle(1,3), thisRobotBundle(1,4), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
            end            
        end
    end
    
    %% Execution + conflict resolution overview
    % Run the robots to reach their assigned racks along the computed paths
    % Once a robot arrives to the first rack, it stays one step at the rack for loading.
    % Then it moves to the next rack.
    % Once all items are loaded, the robot goes to the packing station and stays one step for unloading.
    % If there is no waiting tasks in the queue, the robots goes to the center of the warehouse until new tasks arrive.
    % Locally resolve conflicts while the robots are running: basically, stop and wait to the robots encountered pass. No proactive detour for avoiding dynamic obstacles (robots)
    %  -> This would be the hardest part. If D*-lite solves this problem, that would be great. If not,
    %     Check the cell at the forward direction
    %     If it is occupied, wait in this time step.
    modWhen = 0;
    workingRobotIdx = robotSet((robotSet(:,4)>0),1);%Identify working robots
    for i = 1:length(workingRobotIdx)
        timeRobotMove = timeRobotMove + 1;
        thisRobotIdx = workingRobotIdx(i);
        thisRobotPath = robotPaths{thisRobotIdx};
        thisRobotXY = robotSet(thisRobotIdx, 2:3);
        thisRobotBundle = robotBundles{thisRobotIdx};
        timeInExecution = timeInExecution + size(robotBundlesRecord{thisRobotIdx},1);
        % See the next path thisRobotPath(1,:)
        
        if isequal(thisRobotPath(1,:),thisRobotXY)
            if size(thisRobotPath,1) > 1
                myNextCell = thisRobotPath(2,:);
            else
                myNextCell = thisRobotPath(2,:);
            end
            myCrtCell = thisRobotPath(1,:);
        elseif isempty(thisRobotPath)
            myNextCell = thisRobotXY;
            myCrtCell = thisRobotXY;
        else
            myNextCell = thisRobotPath(1,:);
            myCrtCell = thisRobotPath(1,:);
        end
        
        conflict = false;
        makeDetour = false;        
        % Check other robots' second-top cell in robotPaths{:}
        for j=1:nRobot
            if robotSet(j,4)==0
                yourNextCell = [robotSet(j,2),robotSet(j,3)];
                yourCrtCell = [robotSet(j,2),robotSet(j,3)];
            else
                yourNextPath = robotPaths{j};
                if isequal(yourNextPath(1,:),robotSet(j,2:3))
                    if size(yourNextPath,1) > 1
                        yourNextCell = yourNextPath(2,:);
                    else
                        yourNextCell = yourNextPath(1,:);
                    end
                    yourCrtCell = yourNextPath(1,:);
                elseif isempty(yourNextPath)
                    yourNextCell = [robotSet(j,2),robotSet(j,3)];
                    yourCrtCell = [robotSet(j,2),robotSet(j,3)];
                else
                    yourNextCell = yourNextPath(1,:);
                    yourCrtCell = yourNextPath(1,:);
                end                
            end
            % If there is at least one robot that has the same next cell,
            if (isequal(myNextCell, yourNextCell) || isequal(myNextCell, yourCrtCell) || isequal(myCrtCell, yourNextCell)) && (j ~= thisRobotIdx)
                conflict = true;
                if (j > thisRobotIdx)
                    makeDetour = true;% If its ID is larger than me, I should not move
                end
            end
        end
%%%%%%%%%%%%%%
        % If conflict is false, just the same with the current version
        if (~conflict) || (conflict && ~makeDetour)
            robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
            thisRobotPath(1,:) = [];
            robotPaths{thisRobotIdx} = thisRobotPath;
        end
        
        % If conflict is true and deferMove is true,
        if conflict && makeDetour
            for k=1:nRobot
                if robotSet(k,4)==0
                    gridMap(robotSet(k,2),robotSet(k,3)) = 1;
                else
                    yourNextPath = robotPaths{k};
                    if isequal(yourNextPath(1,:),robotSet(k,2:3))
                        if size(yourNextPath,1) > 1
                            yourNextCell = yourNextPath(2,:);
                        else
                            yourNextCell = yourNextPath(1,:);
                        end
                        yourCrtCell = yourNextPath(1,:);
                    elseif isempty(yourNextPath)
                        yourNextCell = [robotSet(k,2),robotSet(k,3)];
                        yourCrtCell = [robotSet(k,2),robotSet(k,3)];
                    else
                        yourNextCell = yourNextPath(1,:);
                        yourCrtCell = yourNextPath(1,:);
                    end          
                    gridMap(yourNextCell(1),yourNextCell(2)) = 1;
                    gridMap(yourCrtCell(1),yourCrtCell(2)) = 1;
                end
            end
            gridMap(thisRobotXY(1), thisRobotXY(2)) = 0;
            % If conflict is true and deferMove is false, I am the robot with the highest priority.
            % rerun the A* planner with the current location of mine to the task        
            startXY = thisRobotXY;
            if isempty(thisRobotBundle)
                goalXY = stationXY;
            else
                goalXY  = thisRobotBundle(1,3:4);
            end
            gridMap(goalXY(1), goalXY(2)) = 0;%goal location should be free
            thisRobotPath = flipud(A_Star_Search(startXY, goalXY, ~gridMap));
            modWhen = 1;
            robotSet(thisRobotIdx,2:3) = thisRobotPath(1,:);
            if showFig == 2
                delete(hPath(thisRobotIdx))
                hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'co', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','c', 'MarkerFaceColor','c');
            end
            thisRobotPath(1,:) = [];
            robotPaths{thisRobotIdx} = thisRobotPath;
            
        end

        if showFig > 0
            title({['Step ' num2str(tStep)];''})
            delete(hRobot)
            hRobot = plot(robotSet(:,2), robotSet(:,3), 'ro', 'linewidth', 1, 'markersize',21, 'MarkerEdgeColor','r', 'MarkerFaceColor','r')    ;
        end
        if isempty(thisRobotPath)%if the current task is done, now need to see the bundle for the rest of tasks in the bundle
            if robotSet(thisRobotIdx,4)==2
                thisRobotTaskIdx = thisRobotBundle(1,2);
                thisRobotBundle(1,:) = [];%arrived so remove from bundle
                robotBundles{thisRobotIdx} = thisRobotBundle;                
            end
            if showFig == 2
                delete(hPath(thisRobotIdx))
                if robotSet(thisRobotIdx,4)==2
                    delete(hTask(thisRobotTaskIdx))
                end
            end
            if isempty(thisRobotBundle)%if bundle is empty
                if robotSet(thisRobotIdx,4) == 2
                    robotSet(thisRobotIdx,4) = 1;%mark this robot going to packing station
                    %plan path to the station
                    startXY = thisRobotXY;
                    goalXY  = stationXY;
                    thisRobotPath = flipud(A_Star_Search(startXY, goalXY, ~gridMap));
                    modWhen = 2;
                    robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                    if showFig == 2
                        hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                    end            
                elseif robotSet(thisRobotIdx,4) == 1
                    robotSet(thisRobotIdx,4) = 0; %mark this robot idle
                    % At this time, the tasks are completed
                    doneTasksInThisBundle = robotBundlesRecord{thisRobotIdx};
                    for k=1:size(doneTasksInThisBundle,1)
                        taskAll(doneTasksInThisBundle(k,2), 3) = tStep;
                    end
                end                
            else%if there is at least one task remaining in the bundle
                thisRobotTaskIdx = thisRobotBundle(1,2);
                startXY = thisRobotXY;
                goalXY  = thisRobotBundle(1, 3:4);
                thisRobotPath = flipud(astar(startXY, goalXY, ~gridMap));
                modWhen = 3;
                robotPaths{thisRobotIdx} = thisRobotPath; %register path (begins with the crt pose but in the same step the robot shouldn't move->stay one step for path computation)
                if showFig == 2
                    hTask(thisRobotTaskIdx) = plot(thisRobotBundle(1,3), thisRobotBundle(1,4), 'ys', 'linewidth', 1, 'markersize',20, 'MarkerEdgeColor','y', 'MarkerFaceColor','y');
                    hPath(thisRobotIdx) = plot(thisRobotPath(:,1), thisRobotPath(:,2), 'go', 'linewidth', 1, 'markersize',7, 'MarkerEdgeColor','g', 'MarkerFaceColor','g');
                end            
            end
        end
         gridMap = zeros(xSize,ySize);
    end
    timeRobotWait = timeRobotWait + sum((robotSet(:,4)==0));
    if showFig > 0
        drawnow
    end
    %% Increment counter
    tStep = tStep + 1;
end
numCompletedTasks = sum((taskAll(:,3) >= 0));
tp = taskAll((taskAll(:,3) >= 0),:);
tp2 = taskAll((taskAll(:,3) < 0),:);
completionTimeDoneTasks = sum(tp(:,3)-tp(:,2));
taskAll((taskAll(:,3) < 0),3) = tStep;
completionTimeAllTasks = sum(taskAll(:,3)-taskAll(:,2));
tp2(:,3) = tStep;
timeInQueueTasks = timeInQueueTasks + sum(tp2(:,3)-tp2(:,2));
numAllTasks = taskNum;

fprintf('%d robots, %d bundle size, %d tasks done\n', nRobot, bSize, numCompletedTasks)
fprintf('Execution time per task: %.4f\n', timeRobotMove/(numCompletedTasks))
fprintf('Bundling time per task: %.4f\n', timeInBundle/numBundledTasks)
fprintf('Timespan per task (all): %.4f\n', completionTimeAllTasks/numAllTasks)
fprintf('Timespan per task (done): %.4f\n', completionTimeDoneTasks/numCompletedTasks)
fprintf('Queue residing time: %.4f\n', timeInQueueTasks/numAllTasks)
(timeInQueueTasks + timeInBundle + timeInExecution)/numAllTasks
