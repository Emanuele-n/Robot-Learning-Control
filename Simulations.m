close all
clear all
clc

global n

addpath(genpath('./functions'));

n=2;

Rob_Nominal = get_Robot_Nominal();


%% Simulation parameters

% Initial Condition
q0=[0;-pi/2];
q0_dot=[0;0];

% Final Condition
qf=[4*pi;pi/2];
qf_dot=[0;0];

% Gains
Kp=diag([1000,1000]);
Kd=Kp/10;

% Time
tf=1;
step=1e-3;


%% Trajectory

% Desired Trajectory
tic
[qd, qd_dot,qd_dotdot] = get_Trajectory_Desired(q0,q0_dot,[0;0],qf,qf_dot,[0;0],tf,step,n);
toc


%% Simulations

% Computed Torque
tic
[q,q_dot,e] = run_Computed_Torque(Rob_Nominal,q0,q0_dot,qd,qd_dot,qd_dotdot,Kp,Kd,tf,step,n);
toc
        

%% Plotting

figure(1)
tiledlayout(n,2);

for i=1:n

    nexttile
    plot(q(i,:));
    hold on
    plot(qd(i,:),'-.r');
    legend({'Position','Reference'});
    title('Position Link ' + string(i));
    xlabel('t (ms)');
    ylabel('q' + string(i) + ' (rad)');
    grid on

    nexttile
    plot(e(i,:));
    %yline(0,'-.r');
    legend({'Error'});
    title('Error Link ' + string(i));
    xlabel('t (ms)');
    ylabel('e' + string(i) + ' (rad)');
    grid on

end

wait1=input('print?');
set(gcf,'Renderer','Painters');
saveas(gcf,'2RDATACT','svg');

%% Animation

figure(2)
tiledlayout(1,1);

Rob_Nominal.plot([q(1,1) q(2,1)],'trail','-.r')
 
for i = 1:10:tf*(1/step)
   Rob_Nominal.animate([q(1,i) q(2,i)]) 
end

wait2=input('print?');
set(gcf,'Renderer','Painters');
saveas(gcf,'2RPLOTCT','svg');
