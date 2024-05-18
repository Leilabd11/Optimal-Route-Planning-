function [ProblemGrid, map] = ProblemDefinition (Cities)
%function ProblemDefinition (Cities)

% % Define grid and obstacles
gridSize = 100;

% Create grid with all zeros (no obstacles)
grid = zeros(gridSize);

% Define obstacles as filled shapes. An example is defined

rectangle1 = [10,20,30,10]; % [y, x, height, width]
rectangle2 = [10,30,10,10];
rectangle3 = [30,30,10,10];
rectangle4 = [60,45,35,10];
rectangle5 = [10,75,50,10];

% Update grid with obstacles
for x = 1:gridSize
    for y = 1:gridSize
        % Check if current position is inside any obstacle
        if (x >= rectangle1(1) && x <= rectangle1(1) + rectangle1(3) && ...
            y >= rectangle1(2) && y <= rectangle1(2) + rectangle1(4)) || ... 
            (x >= rectangle2(1) && x <= rectangle2(1) + rectangle2(3) && ...
            y >= rectangle2(2) && y <= rectangle2(2) + rectangle2(4)) || ...
            (x >= rectangle3(1) && x <= rectangle3(1) + rectangle3(3) && ...
            y >= rectangle3(2) && y <= rectangle3(2) + rectangle3(4)) || ...
            (x >= rectangle4(1) && x <= rectangle4(1) + rectangle4(3) && ...
            y >= rectangle4(2) && y <= rectangle4(2) + rectangle4(4)) || ...
            (x >= rectangle5(1) && x <= rectangle5(1) + rectangle5(3) && ...
            y >= rectangle5(2) && y <= rectangle5(2) + rectangle5(4))
            grid(x, y) = 1; % Set obstacle
        end
    end
end

ProblemGrid=transpose(grid);

map = occupancyMap(grid, 1);


end
