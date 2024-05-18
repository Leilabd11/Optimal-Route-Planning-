function main
    % Define the Cities variable
    % Uncomment the set of cities you want to use

    % 6 cities
    Cities=[40,90; 40,70; 90,70; 90,30; 25,35; 25,10];

    % 10 cities
    % Cities = [40,90; 40,70; 90,70; 90,30; 25,35; 25,10; 20,60; 80,90; 90,10; 10,10];

    % 20 cities
    %Cities = [40,90; 40,70; 90,70; 90,30; 25,35; 25,10; 20,60; 80,90; 90,10; 10,10; 10,90; 10,5; 70,10; 95,20; 30,95; 50,60; 25,50; 5,30; 60,35; 28,40];

    % 30 cities
    % Cities = [40,90; 40,70; 90,70; 90,30; 25,35; 25,10; 20,60; 80,90; 90,10; 10,10; 10,90; 10,5; 70,10; 92,20; 30,92; 50,60; 25,50; 5,30; 60,35; 75,60; 10,50; 60,20; 70,92; 28,40; 30,70; 55,50; 90,40; 90,92; 5,60; 35,50];

    % 35 cities
    % Cities = [40,90; 40,70; 90,70; 90,30; 25,35; 25,10; 20,60; 80,90; 90,10; 10,10; 10,90; 10,5; 70,10; 94,20; 30,95; 50,60; 25,50; 5,30; 60,35; 75,60; 10,50; 60,20; 70,95; 28,40; 30,70; 55,50; 90,40; 90,95; 5,60; 35,50; 80,25; 15,45; 46,30; 70,80; 35,60];

    % Run the ProblemDefinition function
    [ProblemGrid, map] = ProblemDefinition(Cities);

    % Call the Astar function to get the distance matrix
    d = Astar(Cities, map);
    
    % Solve the E-TSP through optimization (Yalmip)
    [TotalTrajectoryYalmip] = ETSP_Opt_Yalmip(Cities, map, d);
    
    % Solve the E-TSP through Greedy Heuristic
    [TotalTrajectoryGreedy] = ETSP_Greedy(Cities, map, d);

    % Plot the trajectory for Yalmip
    Trajectory_plot(ProblemGrid, TotalTrajectoryYalmip, Cities, 'Experiment solved using A* and E-TSP (via Yalmip)');

    % Plot the trajectory for Greedy
    Trajectory_plot(ProblemGrid, TotalTrajectoryGreedy, Cities, 'Experiment solved using A* and E-TSP (via Greedy heuristic)');

    % Call the DubinsCircles function for the 6-point example
    DubinsCircles(ProblemGrid, Cities, TotalTrajectoryYalmip);
end


function [ProblemGrid, map] = ProblemDefinition(Cities)
    % Define grid and obstacles
    gridSize = 100;

    % Create grid with all zeros (no obstacles)
    grid = zeros(gridSize);

    % Define obstacles as filled shapes
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

    ProblemGrid = transpose(grid);
    map = occupancyMap(grid, 1);
end

function d = Astar(Cities, map)
    N = size(Cities, 1); % Number of cities
    d = zeros(N);

    % Compute distance between each city with A* and save it in matrix d
    for i = 1:N
        for j = 1:N
            if i ~= j % Distance between a city and itself does not make sense
                planner = plannerAStarGrid(map);
                [path, debugInfo] = plan(planner, Cities(i,:), Cities(j,:));
                d(i,j) = debugInfo.PathCost;
            end
        end
    end
end

function [TotalTrajectory] = ETSP_Opt_Yalmip(Cities, map, d)
    % Define decision variables
    N = size(Cities, 1);

    x = binvar(N, N, 'full'); % Binary variable indicating whether a visit to vertex i is followed by a visit to vertex j
    u = intvar(N, 1); % Position of each vertex in the path. Intvar generates a vector of integers

    % Define the objective function
    objective = sum(sum(d .* x));

    % Constraint 1: Visit each node exactly once
    constraints = [];
    constraints = [constraints, sum(x(1,2:N)) == 1]; % Start from the first city
    constraints = [constraints, sum(x(2:N,N)) == 1]; % End at city N
    for i = 1:N-1
        constraints = [constraints, sum(x(i,:)) == 1]; % Outgoing edges (only one)
        constraints = [constraints, sum(x(:,i)) == 1]; % Incoming edges (only one)
    end

    % Constraint 2: Prevent subtours
    for i = 2:N
        constraints = [constraints, 2 <= u(i) <= N]; % Position of each vertex in the path
        for j = 2:N
            constraints = [constraints, u(i) - u(j) + 1 <= (N-1)*(1 - x(i,j))]; % Constraints for preventing subtours
        end
    end
    constraints = [constraints, u(1) == 1]; % Start from the first city

    % Constraint 3: Binary decision variables
    constraints = [constraints, integer(u)];

    % Solve the optimization problem
    optimize(constraints, objective);

    % Retrieve the optimal solution
    optimal_x = value(x);
    optimal_u = value(u);
    optimal_distance = value(objective)

    % Reorder the cities based on the E-TSP order result
    OrderedCities = zeros(N, 2);
    for i = 1:N
        for j = 1:N
            if optimal_u(i) == j
                OrderedCities(j,:) = Cities(i,:);
            end
        end
    end

    % Get the trajectory to plot it
    TotalTrajectory = [];
    PathCost = [];
    planner = plannerAStarGrid(map);

    for i = 1:N-1
        Start = OrderedCities(i,:);
        Goal = OrderedCities(i+1,:);
        
        % Plan the path from Start to Goal
        [path, debugInfo] = plan(planner, Start, Goal); % Path is a matrix with the coordinates of the followed path
        PathCost = [PathCost; debugInfo.PathCost];
        TotalTrajectory = [TotalTrajectory; path];
    end
    
    % Return path to the starting city
    [path, debugInfo] = plan(planner, OrderedCities(N,:), OrderedCities(1,:));
    TotalTrajectory = [TotalTrajectory; path];
    PathCost = [PathCost; debugInfo.PathCost];
end

function [TotalTrajectory] = ETSP_Greedy(Cities, map, distance)
    % Approve the distance table.
    [m, n] = size(distance);

    if m ~= n
        fprintf('tsp_greedy(): Fatal error! The distance matrix D must be square.\n');
        return
    end

    if n < 4
        fprintf('tsp_greedy(): Fatal error! This problem is too small! The number of cities N must be at least 4.\n');
        return
    end

    v = diag(distance);
    test = norm(v);

    if 0.0 < test
        fprintf('tsp_greedy(): Fatal error! The distance matrix D must have zero diagonal.\n');
        return
    end

    cost_best = Inf;
    p_best = randperm(n);

    for start = 1:n
        p = path_greedy(n, distance, start);
        cost = path_cost(n, distance, p);

        if cost < cost_best
            p_best = p;
            cost_best = cost;
        end 
    end

    OrderedCities = zeros(n, 2);
    for i = 1:n
        for j = 1:n
            if p_best(i) == j
                OrderedCities(i,:) = Cities(j,:);
            end
        end
    end

    TotalTrajectory = [];
    planner = plannerAStarGrid(map);

    for i = 1:n-1 
        Start = OrderedCities(i,:);
        Goal = OrderedCities(i+1,:);
        [path, debugInfo] = plan(planner, Start, Goal);
        TotalTrajectory = [TotalTrajectory; path];
    end

    [path, debugInfo] = plan(planner, OrderedCities(n,:), OrderedCities(1,:)); 
    TotalTrajectory = [TotalTrajectory; path];
end

function cost = path_cost(n, distance, p)
    cost = 0.0;
    i1 = n;
    for i2 = 1:n
        cost = cost + distance(p(i1), p(i2));
        i1 = i2;
    end
end

function p = path_greedy(n, distance, start)
    p = zeros(n, 1);
    p(1) = start;

    d = distance(1:n, 1:n);
    d(:, start) = Inf;

    for i = 1:n
        d(i, i) = Inf; 
    end

    from = start;
    for j = 2:n
        [~, to] = min(d(from,:));
        p(j) = to;
        d(:,to) = Inf;
        from = to;
    end
end


function Trajectory_plot(ProblemGrid, TotalTrajectory, Cities, plotTitle)
    figure
    imagesc(ProblemGrid);
    colormap([1 1 1; 0 0 0]);
    hold on 
    scatter(TotalTrajectory(:,1), TotalTrajectory(:,2), 10, 'filled', 'DisplayName', 'A* optimal trajectory')
    scatter(Cities(:,1), Cities(:,2), 'r', 'filled', 'DisplayName', 'Points of Interest')

    NumCities = size(Cities, 1);
    for i = 1:NumCities
        text(Cities(i,1), Cities(i,2) + 2.5, ['P' num2str(i-1)], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    end

    quiver(TotalTrajectory(1:end-1,1), TotalTrajectory(1:end-1,2), diff(TotalTrajectory(:,1)), diff(TotalTrajectory(:,2)), 0, 'color', [0.6350 0.0780 0.1840], 'AutoScale', 'off', 'MaxHeadSize', 20, 'DisplayName', 'Trajectory direction');

    xlim([0, 100]);
    ylim([0, 100]);

    title(plotTitle);
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('show', 'Location', 'northeast');
end


function DubinsCircles(ProblemGrid, Cities, TotalTrajectory)
    %Dubins Circles Main experiment grid size 100x100

    % Plot A*-ETSP trajectories
    figure;
    imagesc(ProblemGrid);
    colormap([1 1 1; 0 0 0]);
    hold on;

    % Scatter plot for Points of Interest
    scatter(Cities(:,1), Cities(:,2),'r','filled', 'DisplayName','Points of Interest');
    hold on;
    
    % Scatter plot for A* optimal trajectory
    scatter(TotalTrajectory(:,1),TotalTrajectory(:,2),10,'b','filled','DisplayName','A* optimal trajectory');
    hold on;
    
    % City Labels
    labels = {'P0', 'P1', 'P2', 'P3', 'P4', 'P5'};
    for i = 1:size(Cities, 1)
        text(Cities(i, 1), Cities(i, 2) - 1, labels{i}, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    end

    xlim([0, 100]); % Assuming grid spans from 0 to 100
    ylim([0, 100]);
    title('Main experiment solved with A*+E-TSP + Dubins circles');
    xlabel('X(m)');
    ylabel('Y(m)');
    legend('show','Location','northeast');

    % Define the start and goal poses for the Dubins paths
    startPoses = [
        40, 90, pi/4;
        90, 70, -pi/6;
        90, 30, -5*pi/6;
        25, 10, -7*pi/8;
        25, 35, -10*pi/24;
        40, 70, pi/8
    ];
    
    goalPoses = [
        90, 70, -pi/6;
        90, 30, -5*pi/6;
        25, 10, -7*pi/8;
        25, 35, -10*pi/24;
        40, 70, pi/8;
        40, 90, -pi
    ];

    % Define the turning radii for the Dubins paths
    turningRadii = [8.5, 12.0, 5, 10.5, 1.2, 11.7];

    % Generate and show the Dubins paths
    for i = 1:length(startPoses)
        dubConnObj = dubinsConnection;
        dubConnObj.MinTurningRadius = turningRadii(i);
        pathSegObj = connect(dubConnObj, startPoses(i,:), goalPoses(i,:));
        show(pathSegObj{1});
    end
end
