clear all

close all

clc

%% parameters

g=9.81;

b=0.2;

m=1.64;

M_1=9.20;

l=1.98;

j_s=(m*l^2)/3;

N=j_s+m*l^2;

M=M_1+m;

T_sim = 3;

x_0_1=0;

x_0_2=0;

x_0_3=0;

x_0_4=0;

x0 = [x_0_1 x_0_2 x_0_3 x_0_4]';

%% SS FOR LINEAR

A_1=[0 1 0 0;

     0 (b*N)/(m^2*l^2-M*N) (m^2*g*l^2)/(m^2*l^2-M*N) 0;

     0 0 0 1;

     0 (-b*m*l)/(m^2*l^2-M*N) (-m*M*g*l)/(m^2*l^2-M*N) 0];

B_1=[0;-N/(m^2*l^2-M*N);0;(m*l)/(m^2*l^2-M*N)];

C_1=eye(4);

D_1=zeros(4,1);

sys_1 = ss(A_1,B_1,C_1,D_1);

%% Trim the Simulink model



% Equilibrium point

theta_bar = 0; 

theta_bar_dot=0;

x_bar = 0;

x_bar_dot = 0;

% Open the nonlinear Simulink model

system = 'nonlinear';

open(system)



% Get the input/output points of the Simulink model

IO = getlinio(system);



% Get the operating point structure for the model.

OP = operspec(system);



% Set both states to be at steady-state at trim.

OP.States(1).SteadyState = 1;

OP.States(2).SteadyState = 1;

OP.States(3).SteadyState = 1;

OP.States(4).SteadyState = 1;

linopt = findopOptions('DisplayReport','off');

% Set the output to be known at v_bar. Output equals state x

OP.Output.Known =  ones(4,1);

OP.Output.y = [x_bar,x_bar_dot,theta_bar,theta_bar_dot]



[OP_POINT,OP_REPORT]=findop(system,OP);



%% Linearize the Simulink model



% Cycle through the parameter domain and linearize at each grid point.

LinearSystem = linearize(system,OP_POINT,IO);

[y,p]=sort(LinearSystem.Statename)

LinearSystem=xperm(LinearSystem,[3;4;1;2])

%% Set nonlinear model to trim values





% Get the state space data from the linearized model

A = LinearSystem.A;

B = LinearSystem.B;

C = LinearSystem.C;

D = LinearSystem.D;



sim('nonlinear_linear',T_sim);

%TASK 3---------------------------------------------------------------------------------

%% pole

x_0_1=1.1;

x_0_2=0.4;

x_0_3=0.1;

x_0_4=0.3;

C=[1 0 0 0];

p = [-2.5 -2.8 -3.9 -4];



Ktest = place(A,B,p);

NxNu = inv([A B; C 0])*[zeros(4,1);1];



Nx = NxNu(1:4,:);

Nu = NxNu(5);

%% Run simulation

T_sim = 50;

sim('nonlinear_poleplace',T_sim);



% Plot simulation results

% Control input, u





%TASK 4---------------------------------------------------------------------------------------------------------------------------

%% Control design

x_0_1=1.1;

x_0_2=0.4;

x_0_3=0.1;

x_0_4=0.3;

Q = [100 0 0 0;

    0 1 0 0;

    0 0 10 0;

    0 0 0 100];



% Compare the results with R = 0.01 and R = 1

R =  1;

x0 = [x_0_1 x_0_2 x_0_3 x_0_4]';

K = lqr(A,B,Q,R)



T_sim = 5;

sim('nonlinear_LQR',T_sim);

    

   

%TASK5-----------------------------------------------------------

%% kamlman filter

x_0_1=-0.01;

x_0_2=0.01;

x_0_3=0.05;

x_0_4=0.1;

Q_w = [1 0 0 0;

    0 10 0 0;

    0 0 1 0;

    0 0 0 10];

R_w = 0.1;

C=[1 0 0 0];

x0 = [x_0_1 x_0_2 x_0_3 x_0_4]';

K_kalman = lqr(A',C',Q_w,R_w);

K_kalman = K_kalman'

sysKF = ss(A-K_kalman*C,[B K_kalman],eye(4),0*[B K_kalman]);



T_sim = 50;

%sim('nonlinear_kalman',T_sim);

%TASK 6-------------------------------------------------------------

%% trajectories



T_sim=50;

sim('nonlinear_trajectory',T_sim);

%task7-------------------------------

%% values

M=M_1*1.05+m;

l=l*1.05;

