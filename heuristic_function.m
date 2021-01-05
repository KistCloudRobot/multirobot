%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Author:Michael Jacob Mathew
% Modified by Changjoo Nam

% The following class makes cell of each grid.
% The code is self explanatory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function heuristics = heuristic_function(grid, goal)
    heuristics = zeros(size(grid));
    for i = 1:size(grid, 1)
        for j = 1:size(grid, 2)
            heuristics(i, j) = sqrt((i-goal(1))^2 + (j-goal(2))^2);
        end
    end
end