function [u] = run_Data_Generation(Rob,q0,q0_dot,qd,qd_dot,qd_dotdot,Kp,Kd,tf,step,n)

    % - - - - - - - - - - - - - - - - - - -
    % input:
    % Rob: Robot
    % q0: Initial Position
    % q0_dot: Initial Velocity
    % qd: Desired Position
    % qd_dot: Desired Velocity
    % qd_dotdot: Desired Acceleration
    % Kp,Kd: Gains
    % tf: Final Time
    % step: Step
    % n: Robot Links
    % - - - - - - - - - - - - - - - - - - -
    % output:
    % u: [u(1),u(2),...,u(n)]
    % - - - - - - - - - - - - - - - - - - -
    
    
    % Initializzation
    q=zeros(n,tf*(1/step));
    q_dot=zeros(n,tf*(1/step));
    e=zeros(n,tf*(1/step));
    q(:,1)=q0;
    q_dot(:,1)=q0_dot;
    e(:,1)=qd(:,1)-q0;
    u(:,1)=[0;0];
    
    for i=1:tf*(1/step)-1
        
        Rob_n = get_Robot_Nominal();

        % Inertia, Coriolis, Gravity vector
        M=Rob.inertia(q(:,i)');
        c=Rob.coriolis(q(:,i)',q_dot(:,i)');
        g=Rob.gravload(q(:,i)')';
        
        Mn=Rob_n.inertia(q(:,i)');
        cn=Rob_n.coriolis(q(:,i)',q_dot(:,i)');
        gn=Rob_n.gravload(q(:,i)')';

        % Computed Torque
        u(:,i+1) = Mn * (qd_dotdot(:,i) + Kp*(qd(:,i) - q(:,i)) + Kd*(qd_dot(:,i) - q_dot(:,i))) + cn*q_dot(:,i) + gn;
       
        % State Update
        q(:,i+1) = q(:,i) + step*(q_dot(:,i));
        q_dot(:,i+1) = q_dot(:,i) + step*(-inv(M)*(c*q_dot(:,i)+g)+M\u(:,i+1));
        e(:,i+1) = qd(:,i) - q(:,i);

    end
    
end
