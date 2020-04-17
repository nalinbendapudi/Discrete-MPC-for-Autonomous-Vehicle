function finalTrajectoryFileName = discreteMPC(intialTrajectoryFileName)
load(intialTrajectoryFileName);  % Contains both states and inputs across all timesteps





finalTrajectoryFileName = 'discreteMPCcontrolTraj.mat';
save(finalTrajectoryFileName,states,inputs) % TODO state and input variables to be put in .mat file
end

