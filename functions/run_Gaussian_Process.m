function [q, q_dot, q_dotdot, e, qr, qr_dot, qr_dotdot, er, var] = run_Gaussian_Process(Rob_Nominal,Rob_Real,q0,q0_dot,q0_dotdot,qr0,qr0_dot,qr0_dotdot,qd,qd_dot,qd_dotdot,Kp,Kd,tf,step,n,sigma_n,sigma,l)

    % - - - - - - - - - - - - - - - - - - -
    % input:
    % Rob_Nominal: Robot (model)
    % Rob_Real: Robot (real)
    % q0: Initial Position
    % q0_dot: Initial Velocity
    % q0_dotdot: Initial Acceleration
    % qd: Desired Position
    % qd_dot: Desired Velocity
    % qd_dotdot: Desired Acceleration
    % Kp,Kd: Gains
    % tf: Final Time
    % step: Step
    % n: Robot Links
    % sigma_n, sigma, l: GP Optimal Parameters
    % - - - - - - - - - - - - - - - - - - -
    % output:
    % q: [q(1),q(2),...,q(n)]
    % q_dot: [q_dot(1),q_dot(2),...,q_dot(n)]
    % q_dotdot: [q_dotdot(1),q_dotdot(2),...,q_dotdot(n)]
    % e: [e(1),e(2),...,e(n)]
    % qr: [q(1),q(2),...,q(n)]
    % qr_dot: [q_dot(1),q_dot(2),...,q_dot(n)]
    % qr_dotdot: [q_dotdot(1),q_dotdot(2),...,q_dotdot(n)]
    % er: [e(1),e(2),...,e(n)]
    % - - - - - - - - - - - - - - - - - - -
    
    % Initializzation
    q=zeros(n,tf*(1/step));
    q_dot=zeros(n,tf*(1/step));
    q_dotdot=zeros(n,tf*(1/step));
    e=zeros(n,tf*(1/step));
    qr=zeros(n,tf*(1/step));
    qr_dot=zeros(n,tf*(1/step));
    qr_dotdot=zeros(n,tf*(1/step));
    er=zeros(n,tf*(1/step));
    var=zeros(1,tf*(1/step));
    q(:,1)=q0;
    q_dot(:,1)=q0_dot;
    q_dotdot(:,1)=q0_dotdot;
    qr(:,1)=qr0;
    qr_dot(:,1)=qr0_dot;
    qr_dotdot(:,1)=qr0_dotdot;
    e(:,1)=qd(:,1)-q0;
    er(:,1)=qd(:,1)-qr0;
    
    bar = waitbar(0, 'Gaussian Process ...');

    for i=1:tf*(1/step)-1
        
        waitbar(i*step/tf, bar);

        % Inertia, Coriolis, Gravity vector
        
        % Nominal Robot (GP)
        M=Rob_Nominal.inertia(q(:,i)');
        c=Rob_Nominal.coriolis(q(:,i)',q_dot(:,i)');
        g=Rob_Nominal.gravload(q(:,i)')';
        
        % Nominal Robot (without GP)
        Mr=Rob_Nominal.inertia(qr(:,i)');
        cr=Rob_Nominal.coriolis(qr(:,i)',qr_dot(:,i)');
        gr=Rob_Nominal.gravload(qr(:,i)')';
        
        % Real Robot (GP)
        M_real=Rob_Real.inertia(q(:,i)');
        c_real=Rob_Real.coriolis(q(:,i)',q_dot(:,i)');
        g_real=Rob_Real.gravload(q(:,i)')';
        
        % Real Robot (without GP)
        Mr_real=Rob_Real.inertia(qr(:,i)');
        cr_real=Rob_Real.coriolis(qr(:,i)',qr_dot(:,i)');
        gr_real=Rob_Real.gravload(qr(:,i)')';

        % Computed Torque
        [eps,v] = run_GP_Predict(sigma_n, sigma, l, [qd(:,i);qd_dot(:,i);qd_dotdot(:,i)]);
       
        u = M * (qd_dotdot(:,i) + Kp*(qd(:,i) - q(:,i)) + Kd*(qd_dot(:,i) - q_dot(:,i))) + c*q_dot(:,i) + g + eps';
        ur = Mr * (qd_dotdot(:,i) + Kp*(qd(:,i) - qr(:,i)) + Kd*(qd_dot(:,i) - qr_dot(:,i))) + cr*qr_dot(:,i) + gr;
                
        % State Update (GP)
        q(:,i+1) = q(:,i) + step*(q_dot(:,i));
        q_dot(:,i+1) = q_dot(:,i) + step*(-inv(M_real)*(c_real*q_dot(:,i)+g_real)+M_real\u);
        q_dotdot(:,i+1) = (-inv(M_real)*(c_real*q_dot(:,i)+g_real)+M_real\u);
        
        e(:,i+1) = qd(:,i) - q(:,i);
        er(:,i+1) = qd(:,i) - qr(:,i);

        % State Update (without GP)
        qr(:,i+1) = qr(:,i) + step*(qr_dot(:,i));
        qr_dot(:,i+1) = qr_dot(:,i) + step*(-inv(Mr_real)*(cr_real*qr_dot(:,i)+gr_real)+Mr_real\ur);
        qr_dotdot(:,i+1) = (-inv(Mr_real)*(cr_real*qr_dot(:,i)+gr_real)+Mr_real\ur);
        
        % Variance Update
        var(1,i+1) = v;
        
    end
   
    close(bar);
    
end

