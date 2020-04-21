function initialTrajectoryFileName = initialTrajectoryGenerator(trackDataFileName, initialState)

%% Inputs

load(trackDataFileName);
leftLine = TestTrack.bl;
rightLine = TestTrack.br;
centerLine = TestTrack.cline;
heading = TestTrack.theta;
%n=10;
n = size(TestTrack.theta,2); % number of coordinates provided in the test-track data

%% Hyper-parameters (Control Parameters)

kp = 0.85;     
kdist = 0.3;

%% Control loop

i=1;    % control input iterator
tp=1;   % track point iterator
currentState = initialState;

while tp < n
    distToNextLine   = distanceToLine([currentState(1),currentState(3),0], [leftLine(:,tp+1)',0], [rightLine(:,tp+1)',0] );
    distToCentreLine = distanceToLine([currentState(1),currentState(3),0], [centerLine(:,tp)',0], [centerLine(:,tp+1)',0]);
    tempCrossProduct = cross( [centerLine(:,tp+1);0]-[centerLine(:,tp);0], [currentState(1);currentState(3);0]-[centerLine(:,tp);0] );
    steer = sum(tempCrossProduct./norm(tempCrossProduct));
        
    if distToNextLine<=1
        tp=tp+1;
    end
    
    % Control Inputs: delta_f and F
    error(i) = (currentState(5)-heading(tp)) + kdist*steer*distToCentreLine;
    delta_f = -kp*error(i);
    
    if currentState(2) < 8
        F = 300;
    else
        F = 75;
    end
    
    currentInput = [delta_f,F];
    inputs(i,:) = currentInput;
    
    Y = singleStepDynamics(currentInput, currentState);
    currentState = Y(end,:);
    i=i+1;
end

%% Outputs

states = multiStepDynamics(inputs, initialState);
initialTrajectoryFileName = 'proportionalControlTraj';
InitialTraj = struct('states',states,'inputs',inputs);
save(initialTrajectoryFileName,'InitialTraj');

end

%% Helper functions

function dist = distanceToLine (point, linePoint1, linePoint2)
    a = linePoint1-linePoint2;
    b = point-linePoint2;
    dist = norm(cross(a,b))/norm(a);
end