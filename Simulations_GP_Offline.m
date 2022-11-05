close all
clear all
clc

addpath(genpath('./functions'));

global n input_training output_training input_active output_active

n=2;

Rob_Nominal = get_Robot_Nominal();
Rob_Real = get_Robot_Real();


%% Simulation parameters

% Time
tf = 1;
step = 1e-3;

% Gains
Kp=diag([1000,1000]);
Kd=Kp/10;


%% GP Parameters

% Active Subset Size
active_p = 50;
%active_p = 10;

% No. Expectation Maximization Steps
EM_steps = 1;

% Initial Guess
sigma_n = 1e-6;
sigma = 1e2;
l = 1e-2 * [1 1];


%% Testing Trajectory (Test Set)

% Initial Condition
q0 = [-pi/2;0];
%q0 = [0;-pi/2];
q0_dot = [0;0];
q0_dot_dot = [0;0];

% Final Condition
qf = [pi/2;pi/2];
%qf = [4*pi;pi/2];
qf_dot = [0;0];
qf_dot_dot = [0;0];

% Quintic Polynomial
tic
[qd, qd_dot,qd_dotdot] = get_Trajectory_Desired(q0,q0_dot,q0_dot_dot,qf,qf_dot,qf_dot_dot,tf,step,n);
toc


%% Initialization

% Training Set
load('data.mat');
dataset = [input;output];
input_training = dataset(1:6,:);
output_training = dataset(7:8,:);

% Active Subset
input_active = [];
output_active = [];

% Active Index Set
I = [];


%% Gaussian Process (OFFLINE)

bar = waitbar(0, "Expectation Maximization Algorithm ...");

disp("GP Initial Parameters:");
disp("sigma_n: " + sigma_n);
disp("sigma: " + sigma);
disp("l: " + l);

for i = 1:EM_steps

    % GP Training
    tic
    [sigma_n_temp, sigma_temp, l_temp, I_temp] = run_GP_Training([input_training;output_training], I, sigma_n, sigma, l, active_p, n, 1);
    toc

    % Gaussian Process
    [q_temp, q_dot_temp, ~, e_temp, qr_temp, qr_dot_temp, ~, er_temp,var_temp] = run_Gaussian_Process(Rob_Nominal,Rob_Real,q0,q0_dot,[0;0],q0,q0_dot,[0;0],qd,qd_dot,qd_dotdot,Kp,Kd,tf,step,n,sigma_n_temp,sigma_temp,l_temp);
    
    if i == 1 || mean(e_temp.^2, "all") < min_error
        
        q = q_temp;
        q_dot = q_dot_temp;
        e = e_temp;
        qr = qr_temp;
        qr_dot = qr_dot_temp;
        er = er_temp;
        var = var_temp;
        min_error = mean(e_temp.^2, "all");
        
        sigma_n = sigma_n_temp;
        sigma = sigma_temp;
        l = l_temp;
        I = I_temp;
        
        if EM_steps ~= 1
            disp("GP Updated Parameters:");
            disp("sigma_n: " + sigma_n);
            disp("sigma: " + sigma);
            disp("l: " + l);
        end

    end
    
    waitbar(i/EM_steps, bar);
    
end

close(bar);

disp("GP Optimal Parameters:");
disp("sigma_n: " + sigma_n);
disp("sigma: " + sigma);
disp("l: " + l);

RMSE = sqrt(mean(mean(e).^2));
disp("");
disp("Testing Trajectory Error:");
disp("RMSE: " + RMSE);



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

clear input;

wait1 = input('print?');

set(gcf,'Renderer','Painters');
saveas(gcf,'2RDATAOFFLINE','svg');


%% Animation
%Initialize video
myVideo = VideoWriter('myVideoFile'); % open video file
myVideo.FrameRate = 10;               % can adjust this
open(myVideo)

figure(2)
tiledlayout(1,1);

Rob_Real.plot([q(1,1) q(2,1)],'trail','-.r')
 
for i = 1:10:tf*(1/step)
   Rob_Real.animate([q(1,i) q(2,i)]) 
   pause(0.01) %Pause and grab frame
   frame = getframe(gcf); %get frame
   writeVideo(myVideo, frame);
end

close(myVideo)

save myVideo.mat

% wait2 = input('print?');
% 
% set(gcf,'Renderer','Painters');
% saveas(gcf,'2RPLOTOFFLINE','svg');


