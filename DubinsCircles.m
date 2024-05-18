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
