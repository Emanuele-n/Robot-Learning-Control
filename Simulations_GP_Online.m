close all
clear all
clc

addpath(genpath('./functions'));

global n input_active output_active input_test output_test input_training output_training

n=2;

Rob_Nominal = get_Robot_Nominal();
Rob_Real = get_Robot_Real();


%% Simulation parameters

% time
tf = 1;
step = 1e-3;

% Data Generation
reduction_step = 1;
optimization_steps = 50;

%% Testing Trajectory

% Initial Condition
q0 = [0;-pi/2];
q0_dot = [0;0];

% Final Condition
qf = [4*pi;pi/2];
qf_dot = [0;0];

% Gains
Kp=diag([1000,1000]);
Kd=Kp/10;


%% GP Parameters

% Number of Samples
active_p = 50;

% Initial Guess
sigma_n = 1e-6;
sigma = 1e2;
l = 1e-2 * [1 1];


%% Parameters Initialization

input_active = [];
output_active = [];
input_test = [];
output_test = [];
input_training = [];
output_training = [];
I = [];


%% Trajectory

% Desired Trajectory
tic
[qd, qd_dot,qd_dotdot] = get_Trajectory_Desired(q0,q0_dot,[0;0],qf,qf_dot,[0;0],tf,step,n);
toc

q=zeros(n,tf*(1/step));
q_dot=zeros(n,tf*(1/step));
q_dotdot=zeros(n,tf*(1/step));
e=zeros(n,tf*(1/step));
qr=zeros(n,tf*(1/step));
qr_dot=zeros(n,tf*(1/step));
qr_dotdot=zeros(n,tf*(1/step));
er=zeros(n,tf*(1/step));
var=zeros(1,tf*(1/step));

%% Gaussian Process (ONLINE)

bar = waitbar(0, 'Online Learning ...');

j = 1;
p = 0;

for k=1:optimization_steps

    tf_step = tf / optimization_steps;
    tf_step_red = tf_step / reduction_step;
    
    w = j + tf*(1/step)/optimization_steps - 1;

    if j == 1
        q0_step = q0;
        q0_dot_step = q0_dot;
        q0_dotdot_step = [0;0];
        qr0_step = q0;
        qr0_dot_step = q0_dot;
        qr0_dotdot_step = [0;0];
    else
        q0_step = q(:,j-1);
        q0_dot_step = q_dot(:,j-1);
        q0_dotdot_step = q_dotdot(:,j-1);
        qr0_step = qr(:,j-1);
        qr0_dot_step = qr_dot(:,j-1);
        qr0_dotdot_step = qr_dotdot(:,j-1);
    end

    qd_step = qd(:,j-p:w-p);
    qd_dot_step= qd_dot(:,j-p:w-p);
    qd_dotdot_step = qd_dotdot(:,j-p:w-p);
  
    p = p + 1;
    
    qd_step_red = qd_step(:,mod(1:size(qd_step,2),reduction_step) == 0);
    qd_dot_step_red= qd_dot_step(:,mod(1:size(qd_dot_step,2),reduction_step) == 0);
    qd_dotdot_step_red = qd_dotdot_step(:,mod(1:size(qd_dotdot_step,2),reduction_step) == 0);
    
    u_nom = run_Data_Generation(Rob_Nominal, q0_step, q0_dot_step, qd_step_red, qd_dot_step_red, qd_dotdot_step_red,Kp,Kd, tf_step_red, step, n);
    u_real = run_Data_Generation(Rob_Real, q0_step, q0_dot_step, qd_step_red, qd_dot_step_red, qd_dotdot_step_red,Kp,Kd, tf_step_red, step, n);

    run_GP_Update_Sets([qd_step_red; qd_dot_step_red;qd_dotdot_step_red], u_real - u_nom);

    [sigma_n, sigma, l, ~] = run_GP_Training([input_training;output_training], I, sigma_n, sigma, l, active_p, n, 0);

    [q(:,j:w), q_dot(:,j:w), q_dotdot(:,j:w), e(:,j:w), qr(:,j:w), qr_dot(:,j:w), qr_dotdot(:,j:w), er(:,j:w), var(1,j:w)] = run_Gaussian_Process(Rob_Nominal, Rob_Real, q0_step, q0_dot_step, q0_dotdot_step,qr0_step, qr0_dot_step, qr0_dotdot_step, qd_step, qd_dot_step, qd_dotdot_step,Kp,Kd,tf_step, step, n, sigma_n, sigma, l);
    
    j = w + 1;
    
    waitbar(k/optimization_steps,bar);
    
end

close(bar);

%% Plotting

figure(1)
tiledlayout(n,2);

for i=1:n

    nexttile
    plot(q(i,:));
    hold on
    plot(qd(i,:),'-.r');
    hold on
    plot(qr(i,:),'-.m');
    hold on
    fill=patch([1:1000 fliplr(1:1000)], [(q(i,:) + var(1,:)) fliplr(q(i,:) - var(1,:))],'k');
    fill.EdgeColor='none';
    set(fill,'Facealpha',.4);
    legend({'Position','Reference','Position without GP','Variance'});
    title('Position Link ' + string(i));
    xlabel('t (ms)');
    ylabel('q' + string(i) + ' (rad)');
    grid on

    
    nexttile
    plot(e(i,:));
    %yline(0,'-.r');
    hold on
    plot(er(i,:),'-.m');
    legend({'Error','Error without GP'});
    title('Error Link ' + string(i));
    xlabel('t (ms)');
    ylabel('e' + string(i) + ' (rad)');
    grid on
    
end

wait1=input('print?');
set(gcf,'Renderer','Painters');
saveas(gcf,'2RDATAONLINE','svg');


%% Animation

figure(2)
tiledlayout(1,1);

Rob_Real.plot([q(1,1) q(2,1)],'trail','-.r')
 
for i = 1:10:tf*(1/step)
   Rob_Real.animate([q(1,i) q(2,i)]) 
end

wait2=input('print?');
set(gcf,'Renderer','Painters');
saveas(gcf,'2RPLOTONLINE','svg');
