clear 
close all

trackDataFileName = 'TestTrack';
initialState = [287,5,-176,0,2,0];

initialTrajectoryFileName = initialTrajectoryGenerator(trackDataFileName,initialState);
disp('Initial Trajectory Generated');
finalTrajectoryFileName = discreteMPC(initialTrajectoryFileName,initialState);
disp('MPC Trajectory Generated');

%% plots
load(trackDataFileName);
leftLine = TestTrack.bl;
rightLine = TestTrack.br;

load(initialTrajectoryFileName);
initialTraj_X = InitialTraj.states(:,1);
initialTraj_Y = InitialTraj.states(:,3);

load(finalTrajectoryFileName);
finalTraj_X = FinalTraj.states(:,1);
finalTraj_Y = FinalTraj.states(:,3);

figure(1)
title('Initial Trajectory - Using Proportional Controller')
hold on
plot(initialTraj_X, initialTraj_Y, 'r','LineWidth',2);
plot(leftLine(1,:), leftLine(2,:), 'k','LineWidth',1);
plot(rightLine(1,:),rightLine(2,:),'k','LineWidth',1);
legend('Trajectory','Left Border','Right Border','Location','NorthWest')
hold off

figure(2)
title('Final Trajectory - Using MPC Controller')
hold on
plot(finalTraj_X,   finalTraj_Y,   'r','LineWidth',2);
plot(leftLine(1,:), leftLine(2,:), 'k','LineWidth',1);
plot(rightLine(1,:),rightLine(2,:),'k','LineWidth',1);
legend('Trajectory','Left Border','Right Border','Location','NorthWest')
hold off
