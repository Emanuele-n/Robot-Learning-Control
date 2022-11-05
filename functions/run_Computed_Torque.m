function [q, q_dot, e] = run_Computed_Torque(Rob,q0,q0_dot,qd,qd_dot,qd_dotdot,Kp,Kd,tf,step,n)

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
    % q: [q(1),q(2),...,q(n)]
    % q_dot: [q_dot(1),q_dot(2),...,q_dot(n)]
    % e: [e(1),e(2),...,e(n)]
    % - - - - - - - - - - - - - - - - - - -

    % Initialization
    q=zeros(n,tf*(1/step));
    q_dot=zeros(n,tf*(1/step));
    e=zeros(n,tf*(1/step));
    q(:,1)=q0;
    q_dot(:,1)=q0_dot;
    e(:,1)=qd(:,1)-q0;

    bar = waitbar(0, 'Trajectory Planning ...');
    
    for i=1:tf*(1/step)-1

        waitbar(i*step/tf, bar);
        
        % Inertia, Coriolis, Gravity vector
        M=Rob.inertia(q(:,i)');
        c=Rob.coriolis(q(:,i)',q_dot(:,i)');
        g=Rob.gravload(q(:,i)')';

        % Computed Torque
        u = M * (qd_dotdot(:,i) + Kp*(qd(:,i) - q(:,i)) + Kd*(qd_dot(:,i) - q_dot(:,i))) + c*q_dot(:,i) + g;

        % State update
        q(:,i+1) = q(:,i) + step*(q_dot(:,i));
        q_dot(:,i+1) = q_dot(:,i) + step*(-inv(M)*(c*q_dot(:,i)+g)+M\u);
        e(:,i+1) = qd(:,i) - q(:,i);

    end
    
    close(bar)

end