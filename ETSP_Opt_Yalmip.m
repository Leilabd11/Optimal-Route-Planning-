function [TotalTrajectory, optimal_distance] = ETSP_Opt_Yalmip(Cities, map, d)
% Define decision variables
N=size(Cities,1);

x = binvar(N,N,'full'); % Binary variable indicating whether a visit to vertex i is followed by a visit to vertex j
u = intvar(N,1); % Position of each vertex in the path. Intvar generates a vector of integers

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
optimal_u = value(u)
optimal_distance = value(objective)

%Reorder the cities based on the E-TSP order result
for i=1:N
    for j=1:N
        if optimal_u(i)==j
            OrderedCities(j,:)=Cities(i,:);
        end
    end
end

%Get the trajectory to plot it

%Initialize variable
TotalTrajectory=[];
PathCost=[];
planner = plannerAStarGrid(map);

for i = 1:N-1 
    Start = OrderedCities(i,:);
    Goal = OrderedCities(i+1,:); 
    
    % Plan the path from Start to Goal
    [path, debugInfo] = plan(planner, Start, Goal); %Path is a matrix with the coordinates of the followed path
     PathCost=[PathCost;debugInfo.PathCost];
    TotalTrajectory=[TotalTrajectory;path];

end
%Return path to the starting city
[path, debugInfo] = plan(planner, OrderedCities(N,:), OrderedCities(1,:)); 
TotalTrajectory=[TotalTrajectory;path];
PathCost=[PathCost;debugInfo.PathCost];


end