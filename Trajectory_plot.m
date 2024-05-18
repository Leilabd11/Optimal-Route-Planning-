function Trajectory_plot (ProblemGrid, TotalTrajectory, Cities)
figure
imagesc(ProblemGrid);
colormap([1 1 1; 0 0 0]);
hold on 
scatter(TotalTrajectory(:,1),TotalTrajectory(:,2),10,'filled','DisplayName','A* optimal trajectory')
hold on
scatter(Cities(:,1),Cities(:,2),'r','filled', 'DisplayName','Points of Interest')
hold on

NumCities=size(Cities);

for i = 1:NumCities
    labels{i} = ['P' num2str(i-1)];
    text(Cities(i,1), Cities(i,2)+2.5, labels{i}, 'VerticalAlignment', 'bottom','HorizontalAlignment','left');
end
 
hold on
TotalTrajectory1=TotalTrajectory(:,1);
TotalTrajectory2=TotalTrajectory(:,2);

quiver(TotalTrajectory1(1:end-1), TotalTrajectory2(1:end-1),diff(TotalTrajectory(:,1)),diff(TotalTrajectory(:,2)),0,'color',[0.6350 0.0780 0.1840],'AutoScale','off','MaxHeadSize',20,'DisplayName','Trajectory direction');

xlim([0, 100]);
ylim([0, 100]);

title('Experiment solved using A* and E-TSP (via Yalmip)');
xlabel('X(m)');
ylabel('Y(m)');
legend('show','Location','northeast');

end