trackDataFileName = 'TestTrack.mat';
intialState = [287,5,-176,0,2,0];

intialTrajectoryFileName = intialTrajectoryGenerator(trackData,intialState);
finalTrajectoryFileName = mpc(initialTrajectoryFileName);

%% plots etc...

