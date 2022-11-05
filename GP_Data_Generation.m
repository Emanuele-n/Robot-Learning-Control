close all
clear all
clc

global n

addpath(genpath('./functions'));

n=2;

Rob_Nominal = get_Robot_Nominal();
Rob_Real = get_Robot_Real();


%% Simulation parameters

% Initial Condition
q0_DG=[-pi/2 0 1 pi/2 -1 pi/3;0 -pi 5 pi/2 pi pi/2];

% Final Condition
qf_DG=[pi/2 0 3.2 -pi/2 pi 4*pi;pi/2 4*pi/3 -2 1 -1 2*pi];

% Time
tf=1;
step=1e-3;

% Gains
Kp=diag([1000,1000]);
Kd=Kp/10;

% Data Generation
trajectories = size(q0_DG,2);
reduction_step = 10;


%% Data Generation

tic
output_full=zeros(n,tf*(1/step)*trajectories);
input_full=zeros(3*n,tf*(1/step)*trajectories);
output = zeros(n,tf*(1/(reduction_step*step))*trajectories);
input = zeros(3*n,tf*(1/(reduction_step*step))*trajectories);

bar = waitbar(0,'Data generation ...');

w=1;

for j=1:1:trajectories

    waitbar(j/length(q0_DG),bar);

    q0j = q0_DG(:,j);
    q0j_dot=[0;0];
    qfj = qf_DG(:,j);
    qfj_dot=[0;0];

    [qdj, qdj_dot,qdj_dotdot] = get_Trajectory_Desired(q0j,q0j_dot,[0;0],qfj,qfj_dot,[0;0],tf,step,n);

    uj_nom = run_Data_Generation(Rob_Nominal,q0j,q0j_dot,qdj,qdj_dot,qdj_dotdot,Kp,Kd,tf,step,n);
    uj_real = run_Data_Generation(Rob_Real,q0j,q0j_dot,qdj,qdj_dot,qdj_dotdot,Kp,Kd,tf,step,n);

    output_full(1:n,((j-1)*(tf*(1/step))+1):(j*(tf*(1/step)))) = uj_real-uj_nom;
    input_full(1:n,((j-1)*(tf*(1/step))+1):(j*(tf*(1/step)))) = qdj;
    input_full(n+1:2*n,((j-1)*(tf*(1/step))+1):(j*(tf*(1/step)))) = qdj_dot;
    input_full(2*n+1:3*n,((j-1)*(tf*(1/step))+1):(j*(tf*(1/step)))) = qdj_dotdot;

    for k=((j-1)*(tf*(1/step))+1):reduction_step:(j*(tf*(1/step)))
        input(:,w) = input_full(:,k);
        output(:,w) = output_full(:,k);
        w = w + 1;
    end
end

close(bar);
save('data.mat','input','output');
toc
        

%% Clear

clear c i j w k output_full input_full q0j q0j_dot qfj qfj_dot qdj qdj_dot qdj_dotdot uj_nom uj_real
