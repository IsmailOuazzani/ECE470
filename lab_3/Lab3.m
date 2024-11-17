% DH Table Initialization for Puma 560
DH_puma560 = [0 pi/2 76 0
    43.23 0 -23.65 0
    0 pi/2 0 0
    0 -pi/2 43.18 0
    0 pi/2 0 0
    0 0 20 0];

% Build the robot model
myrobot = mypuma560(DH_puma560);


%% 3.1 Attractive Field
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=100*[-1; 3; 3;]/4;
q1 = inverse(H1,myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
tau = att(q1,q2,myrobot)


%% 3.2 Motion Planning without Obstacles
q2(4) = pi;
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)


%% 3.3 Motion Planning with Obstacles
setupobstacle % load obstacles into workspace

q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1}) % This tests the torque for the cylinder obstacle

q = [pi/2 pi 1.2*pi 0 0 0]';
tau = rep(q,myrobot,obs{6}) % This tests the torque for the sphere obstacle

% Plot the robot and the obstacles during a test trajectory
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off


