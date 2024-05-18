function d = Astar(Cities,map)

N=size(Cities,1);%num cities
d=zeros(N);

% Compute distance between each city with A* and save it in matrix d
for i=1:N
    for j=1:N
        if i~=j %for i not equal to j, distance between a city and itself does not make sense
            planner = plannerAStarGrid(map);
            [path, debugInfo] = plan(planner, Cities(i,:), Cities(j,:));
            d(i,j)=debugInfo.PathCost;
        end
    end
end

end