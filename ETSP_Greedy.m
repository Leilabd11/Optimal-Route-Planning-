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
