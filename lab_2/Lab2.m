% DH Table Initialization (values computed in prelab)
DH_puma560 = [25 pi/2 400 0
    315 0 0 0
    35 pi/2 0 0
    0 -pi/2 365 0
    0 pi/2 0 0
    -296.23 0 161.44 0];

% Build the robot model
myrobot = mykuka(DH_puma560);

% Testing forward kinematics
H = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4], myrobot);

% Testing inverse kinematics
q = inverse_kuka(H, myrobot)