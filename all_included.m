clear;clc;
data = load('TestTrack.mat');
leftb = data.TestTrack.bl;
rightb = data.TestTrack.br;
centerline = data.TestTrack.cline;
heading = data.TestTrack.theta;
z0=[287,5,-176,0,2,0];
z_temp = z0;
tp=1;
%kd=0.1;
kp=0.85;
kdist=0.3;
tf=220;
dt=0.01;
i = 1;

while tp < 246 
 
    dist = point_to_line([z_temp(1),z_temp(3),0],[leftb(:,tp+1)',0],[rightb(:,tp+1)',0]);
    perp_dist=point_to_line([z_temp(1),z_temp(3),0],[centerline(:,tp)',0],[centerline(:,tp+1)',0]);
    steer=sum(cross([centerline(:,tp+1);0]-[centerline(:,tp);0],[z_temp(1);z_temp(3);0]-[centerline(:,tp);0])./norm(cross([centerline(:,tp+1);0]-[centerline(:,tp);0],[z_temp(1);z_temp(3);0]-[centerline(:,tp);0])));
    if dist<=1
        tp=tp+1;
    end
    e(i)=(z_temp(end,5)-heading(tp))+kdist*steer*perp_dist;
    delta_f=-kp*e(i);
    
   % F = 1000;
    
    if (z_temp(end,1) > 1430)
        F = 4999;
%         if(z_temp(end,2) < 6)
%             F = 150;
%         else
%             F = 0;
%         end
    else
        if(z_temp(end,2) < 8)
            F = 300;
        else
            F = 75;
        end
    end
    
    ROB535_ControlsProject_part1_input(i, :) = [delta_f, F] ;
    
    [Y,T]=forwardIntegrateControlInput1_mod(ROB535_ControlsProject_part1_input(i, :),z_temp);
    z_temp = Y(end, :);
    i = i +1;
end
ROB535_ControlsProject_part1_input = [ROB535_ControlsProject_part1_input;ROB535_ControlsProject_part1_input(end-30:end,:)];
[Y,T]=forwardIntegrateControlInput(ROB535_ControlsProject_part1_input);
figure(1)
plot(Y(:,1),Y(:,3));
hold on
plot(leftb(1,:),leftb(2,:));
hold on
plot(rightb(1,:),rightb(2,:));
plot([leftb(1,end),rightb(1,end)],[leftb(2,end),rightb(2,end)]);

figure(2)
plot(Y(:,2))

function d = point_to_line(pt, v1, v2)
      a = v1 - v2;
      b = pt - v2;
      d = norm(cross(a,b)) / norm(a);
end

function [Y,T]=forwardIntegrateControlInput(U,x0)
% function [Y] = forwardIntegrateControlInput(U,x0)
% 
% Given a set of inputs and an initial condition, returns the vehicles
% trajectory. If no initial condition is specified the default for the track
% is used.
% 
%  INPUTS:
%    U           an N-by-2 vector of inputs, where the first column is the
%                steering input in radians, and the second column is the 
%                longitudinal force in Newtons.
%    
%    x0          a 1-by-6 vector of the initial state of the vehicle.
%                If not specified, the default is used
% 
%  OUTPUTS:
%    Y           an N-by-6 vector where each column is the trajectory of the
%                state of the vehicle
%
%    T           a 1-by-N vector of time stamps for each of the rows of Y.
% 
%  Written by: Sean Vaskov
%  Created: 31 Oct 2018
%  Modified: 6 Nov 2018

%  if initial condition not given use default
if nargin < 2
    x0 = [287,5,-176,0,2,0] ;
end

%generate time vector
T=0:0.01:(size(U,1)-1)*0.01;


%Solve for trajectory      
[~,Y]=ode45(@(t,x)bike(t,x,T,U),T,x0);
end

function [Y,T]=forwardIntegrateControlInput1_mod(U,x0)

if nargin < 2
    x0 = [287,5,-176,0,2,0] ;
end

%generate time vector
T=[0,0.01];%:(size(U,1)-1)*0.01;


%Solve for trajectory      
[~,Y]=ode45(@(t,x)bike_Mod(t,x,T,U),T,x0);
end

function dzdt=bike(t,x,T,U)
%constants
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=0.7;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
g=9.806;


%generate input functions
delta_f=interp1(T,U(:,1),t,'previous','extrap');
F_x=interp1(T,U(:,2),t,'previous','extrap');

%slip angle functions in degrees
a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*F_x)^2+(F_yr^2));
F_max=0.7*m*g;

if F_total>F_max
    
    F_x=F_max/F_total*F_x;
  
    F_yr=F_max/F_total*F_yr;
end

%vehicle dynamics
dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end

function dzdt=bike_Mod(t,x,T,U)
%constants
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=0.7;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
g=9.806;

delta_f = U(1);
F_x = U(2);

%generate input functions
% delta_f=interp1(T,U(:,1),t,'previous','extrap');
% F_x=interp1(T,U(:,2),t,'previous','extrap');

%slip angle functions in degrees
a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*F_x)^2+(F_yr^2));
F_max=0.7*m*g;

if F_total>F_max
    
    F_x=F_max/F_total*F_x;
  
    F_yr=F_max/F_total*F_yr;
end

%vehicle dynamics
dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end