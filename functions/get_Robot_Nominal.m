function Rob = get_Robot_Nominal()

    Links(1)=Revolute('d',0,'a',2,'alpha',0);
    Links(2)=Revolute('d',0,'a',1,'alpha',0);

    % Centers of mass positions array w.r.t. frame i
    Links(1).r=[0 0 0];
    Links(2).r=[0 0 0];

    % Link mass (kg)
    Links(1).m=1;
    Links(2).m=1;

    % Inertia tesor
    Links(1).I=eye(3);
    Links(2).I=eye(3);

    Rob=SerialLink(Links);
    Rob.name='2R';
    Rob.base=[1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1];

end