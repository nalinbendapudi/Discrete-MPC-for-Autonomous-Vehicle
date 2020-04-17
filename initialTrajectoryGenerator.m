function intialTrajectoryFileName = initialTrajectoryGenerator(trackDataFileName, intialState)
load(trackDataFileName);






initialTrajectoryFileName = 'proportionalControlTraj.mat';
save(intialTrajectoryFileName,states,inputs) % TODO state and input variables to be put in .mat file
end

