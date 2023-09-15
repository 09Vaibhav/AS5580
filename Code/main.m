clc;clear;close all
global Nn Dmat x_node traj_model save_state p obs Enhancements

Enhancements =0;
if Enhancements ==1
obs = [0,2];%                      0 2
end

traj_model = 1;
save_state = 1;

if (traj_model == 1) % Circle
    r=1; % radius                       1
    p=[r]; 
elseif (traj_model ==2) % Spiral 
    a=1;   % Constant radius part       1
    b=0.1; % Coeff of variable radius   0.1
    c=1;   % No of spirals              1
    p=[a,b,c]; 
elseif (traj_model ==3) % Elliptical
    a=2;                % 2
    b=1;                % 1
    p=[a,b];
elseif (traj_model ==4)
    a=1;
    b=tan(60*pi/180);
    p=[a,b];
end

Nn = 41;

[x_node,Dmat] = legDc(Nn-1);
x_node = 1-x_node;
if traj_model == 1
    load('Circular_traj.mat')
elseif traj_model == 2
    load('Circular_traj.mat')
elseif traj_model == 3
    load('Circular_traj.mat')
end
% initial Guess array
x_i  = zeros(Nn,1); 
y_i  = zeros(Nn,1);
X_i  = zeros(Nn,1);
x_ei = zeros(Nn,1);
y_ei = zeros(Nn,1);
X_ei = zeros(Nn,1);
l_i  = zeros(Nn,1);
u_i  = zeros(Nn,1);
Tf_i = 20;

% tvec_i = guess(:,1);
% x_i = guess(:,2);
% y_i = guess(:,3);
% X_i = guess(:,4);
% x_ei = guess(:,5);
% y_ei = guess(:,6);
% X_ei = guess(:,7);
% l_i = guess(:,8);
% u_i = guess(:,9);
% Tf_i = tvec_i(end);
Z_0 = [x_i;y_i;X_i;x_ei;y_ei;X_ei;l_i;u_i;Tf_i];
figure(1)
plot(x_i,y_i)

%lb
x_l  =  -500*ones(Nn,1);
y_l  =  -500*ones(Nn,1);
X_l  =  zeros(Nn,1);
x_el = -50*ones(Nn,1); 
y_el = -50*ones(Nn,1);
X_el = -75*pi/180*ones(Nn,1);
l_l  = zeros(Nn,1);
u_l  = -75*pi/180*ones(Nn,1);
Tf_l = 0.1;

lb = [x_l;y_l;X_l;x_el;y_el;X_el;l_l;u_l;Tf_l];

%ub
x_u  =  500*ones(Nn,1);
y_u  =  500*ones(Nn,1);
X_u  =  2*pi*ones(Nn,1);
x_eu =  50*ones(Nn,1); 
y_eu =  50*ones(Nn,1);
X_eu =  75*pi/180*ones(Nn,1);
l_u  =  ones(Nn,1);
u_u  =  75*pi/180*ones(Nn,1);
Tf_u = 100;

ub = [x_u;y_u;X_u;x_eu;y_eu;X_eu;l_u;u_u;Tf_u];

A = []; % No other constraints
b = [];
Aeq = [];
beq = [];

 options =  optimoptions ('fmincon','Display','Iter','OptimalityTolerance',1e-08,'StepTolerance',...
     1e-08, 'ConstraintTolerance' ,1e-08, 'MaxIterations', 5000,'MaxFunctionEvaluations',1200000,'Algorithm', 'sqp');
[Z, costval, exitflag, output] = fmincon(@(Z)Costfunc(Z), Z_0, A, b,...
    Aeq, beq, lb, ub, @(Z)C_fun(Z),options);

x = Z(1:Nn) ;
y = Z(Nn +1:2*Nn) ;
X = Z(2*Nn +1:3*Nn) ;
xe = Z(3*Nn+1:4*Nn) ;
ye = Z(4*Nn+1:5*Nn) ;
Xe = Z(5*Nn +1:6* Nn) ;
l = Z(6*Nn+1:7*Nn) ;
mu = Z(7*Nn+1:8*Nn) ;
Tf = Z(end);

tvec = Tf/2*(x_node);
%%
figure(2)
plot(x,y,"*")
hold on
plot(x-(xe.*cos(X-Xe)-ye.*sin(X-Xe)),y-(ye.*cos(X-Xe)+xe.*sin(X-Xe)),"o")

%%
figure(4)
plot(tvec,X*180/pi)
hold on
plot(tvec,(X-Xe)*180/pi)

%%
figure(5)
plot(tvec,mu*180/pi)
xlabel("Time")
ylabel("Bank angle")
%%
figure(6)
plot(x-(xe.*cos(X-Xe)-ye.*sin(X-Xe)),y-(ye.*cos(X-Xe)+xe.*sin(X-Xe)))
hold on
plot(x,y)
hold on
plot(0,2,"o")
xlabel("X")
ylabel("Y")
legend("Desire Trajectory","Actual trajectory")

%%
guess= [tvec;x;y;X;xe;ye;Xe;l;mu];
guess = reshape(guess,Nn,9);
if save_state ==1
    save("Elliptical_traj.mat",'guess')
end






