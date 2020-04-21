function finalTrajectoryFileName = discreteMPC(initialTrajectoryFileName,initialState)

%% Inputs
load(initialTrajectoryFileName);
Y_ref = InitialTraj.states;
U_ref = InitialTraj.inputs;

% Adding 5% noise in reference trajectory (This doesn't test our MPC
% controller since MPC tracks reference traj regardless of the accuracy of
% reference trajectory)
% Y_ref = Y_ref.*(1+randn(size(Y_ref))/20);
% U_ref = U_ref.*(1+randn(size(U_ref))/20);

%% Reduced-order Dynamic Model

nstates = 5;
ninputs = 1;

% Vehicle Parameters
a   =  1.14;		% distance c.g. to front axle (m) 
L   =  2.54;		% wheel base (m)
m   =  1500;		% mass (kg)
Iz  =  2420.0;	    % yaw moment of inertia (kg-m^2)
b=L-a;              % distance of c.g to rear axel (m) 
g=9.81;
u_0=8;

% Tire forces
B_tf=10;
C_tf=1.3;
D_tf=1;
E_tf=0.97;

% Timespan for all simulations
dt = 0.01;
T=0:0.01:size(U_ref,1)*dt;
U_ref = U_ref';
Y_ref = Y_ref';
input_range = [-0.5, 0.5];

% The front and rear cornerning stiffness
Ca_r= (a*B_tf*C_tf*D_tf*m*g)/L;
Ca_f= (b*B_tf*C_tf*D_tf*m*g)/L;

% Linearizing the bicycle model about the trajectory generated using the P controller

A =   @(i)eye(5) + dt*  [0, 0, -sin(Y_ref(5,i)), -Y_ref(2,i)*sin(Y_ref(5,i)) - Y_ref(4,i)*cos(Y_ref(5,i)), 0;...
         0, 0, cos(Y_ref(5,i)), -Y_ref(4,i)*sin(Y_ref(5,i))+Y_ref(2,i)*cos(Y_ref(5,i)), 0;...
         0, 0, 0, 0, 1;...
         0, 0,-(Ca_r+a*Ca_f*cos(U_ref(1,i)))/(m*Y_ref(2,i)), 0, -((Ca_r*b + Ca_f*a*cos(U_ref(1,i)))/(m*Y_ref(2,i)))-Y_ref(1,i);...
         0, 0,(Ca_r*b - a*Ca_f*cos(U_ref(1,i)))/(Iz*Y_ref(2,i)), 0, -((b^2*Ca_r)+ (a^2*Ca_f*cos(U_ref(1,i))))/(Iz*Y_ref(2,i))];

B =     @(i) dt*[0;...
          0;...
          Ca_f*(cos(U_ref(1,i))-U_ref(1,i)*sin(U_ref(1,i))+((Y_ref(4,i)+a*Y_ref(6,i))*sin(U_ref(1,i))/Y_ref(2,i)))/m;...
          0;...
          a*Ca_f*(cos(U_ref(1,i))-U_ref(1,i)*sin(U_ref(1,i))+((Y_ref(4,i)+a*Y_ref(6,i))*sin(U_ref(1,i)))/Y_ref(2,i))];
      
%% Hyper-parameters

npred = 10; % Number of time-steps in Prediction horizon
Q = [10,10,0.5,500,1];  % state error 
R = 15;              % control input

%% Control Loop

statesTranspose = initialState';
for i = 1:length(T)-1
    disp('##################################################');
    i
    disp('##################################################');
    
npred1 = min([npred,(length(T)-i)]);
Ndec1= (nstates*(npred1+1))+(ninputs*npred1);
% adding noise to the state matrix
% erroneousState = statesTranspose(1:nstates+1, i)+ initialState'.*(0.05*randn(size(initialState')));
% initcond(:,i) = erroneousState - Y_ref(1:nstates+1, i);
initcond(:,i) = statesTranspose(1:nstates+1, i) - Y_ref(1:nstates+1, i);

% beq = zeros([nstates*(npred1+1),1]);
% Aeq = zeros([nstates*(npred1+1),(Ndec1)]);
xsize = nstates*(npred1+1);

stateindx = nstates+1:nstates:xsize;
inputindx = xsize+1:ninputs:Ndec1;
initialindx = i;

[Aeq, beq] = eq_cons(A,B,stateindx,nstates,initialindx,ninputs,inputindx,npred1,Ndec1,initcond([1,3:6],i));

[Lb,Ub]= bound_cons(initialindx,U_ref,input_range,xsize,Ndec1,npred1,nstates);
Lb = Lb';
Ub = Ub';

H= diag([repmat(Q,[1,npred1+1]), repmat(R,[1,npred1])]);
f = zeros([Ndec1,1]);

[Z, ~] = quadprog(H,f,[],[],Aeq,beq,Lb,Ub);
U_mpc = Z(((nstates*(npred1+1))+1): ((nstates*(npred1+1))+ninputs));
inputs(:,i) = U_ref(:,i)- [U_mpc;0];

Y = singleStepDynamics(inputs(:,i),statesTranspose(1:nstates+1, i)');
Y = Y';
statesTranspose(1:nstates+1, i+1) = Y(1:nstates+1,end);
end

%% Outputs

inputs = inputs';
states = multiStepDynamics(inputs, initialState);
finalTrajectoryFileName = 'discreteMPCcontrolTraj.mat';
FinalTraj = struct('states',states,'inputs',inputs);
save(finalTrajectoryFileName,'FinalTraj');

end

%% Helper Functions

function [Aeq1,beq1]=eq_cons(A,B,stateindx,nstates,initialindx,ninputs,inputindx,npred2,Ndec2,init_cond1)
% Aeq*z=beq
% initial_idx specifies the time index of initial condition from the reference trajectory 
% A and B are function handles above
Aeq1(1:nstates,1:nstates)=   eye(nstates); 
beq1(1:nstates) = init_cond1(1:nstates);
beq1(nstates+1:(nstates*(npred2+1)))= zeros(((nstates*(npred2+1))-nstates),1);
for i=1:npred2
    Aeq1(stateindx(i):stateindx(i)+nstates-1,stateindx(i):stateindx(i)+nstates-1)= -eye(nstates);
    Aeq1(stateindx(i):stateindx(i)+nstates-1,stateindx(i)-nstates:stateindx(i)-1)=A(initialindx + i -1);
    Aeq1(stateindx(i):stateindx(i)+nstates-1,inputindx(i):inputindx(i)+ninputs-1)=B(initialindx + i -1);
end
beq1 = beq1';
end

function [Lb,Ub]=bound_cons(idx,U_ref,input_range,xsize,Ndec2,npred2,nstates)
% initial_idx is the index along uref the initial condition is at
Ub(1:nstates*(npred2+1)) = inf;
Lb(1:nstates*(npred2+1)) = -inf;
Ub(xsize+1:Ndec2)= input_range(1,2) - U_ref(1,idx:idx +npred2 -1);
% Ub(xsize+2:2:Ndec2)= input_range(2,2) - U_ref(2,idx:idx +npred2 -1);
Lb(xsize+1:Ndec2)= input_range(1,1)-U_ref(1,idx:idx +npred2 -1);
% Lb(xsize+2:2:Ndec2)= input_range(2,1)-U_ref(2,idx:idx +npred2 -1);
end 
