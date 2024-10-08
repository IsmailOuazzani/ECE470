
%% 4.1
% Definition of robot structure

% DH Table Initialization (values computed in prelab)
DH_puma560 = [0 pi/2 76 0
    43.23 0 -23.65 0
    0 pi/2 0 0
    0 -pi/2 43.18 0
    0 pi/2 0 0
    0 0 20 0];

% Build the robot model
myrobot = mypuma560(DH_puma560);

%% 4.2 Plot a sample joint space trajectory

% Initialize the joint variable vectors
q = [linspace(0, pi, 200)
    linspace(0, pi/2, 200) 
    linspace(0, pi, 200)
    linspace(pi/4, 3*pi/4, 200)
    linspace(-pi/3, pi/3, 200)
    linspace(0, 2*pi, 200)];
q = transpose(q);

% Plot the robot evolution
plot(myrobot, q)

%% 4.3 Forward Kinematics

% Vector to store end effector positions
o = zeros([200,3]);

% Loop through each configuration in q
for i = 1:200
    % Compute homogenous transformation matrix for ith joint configuration
    H =  forward(q(i,:), myrobot);
    % Extract end effector position from H
    o(i,:) = H(1:3, 4);
end

% Plot the trajectory of the end-effector
plot3(o(:,1), o(:,2), o(:,3), 'r')
hold on
plot(myrobot, q)
hold off

%% 4.4 Inverse Kinematics

% First we test that our inverse kinematic function is working as expected.
test_H = [cos(pi/4) -sin(pi/4) 0 20
    sin(pi/4) cos(pi/4) 0 23
    0 0 1 15
    0 0 0 1];
inverse(test_H, myrobot) % expected: [ -0.0331 -1.0667 1.0283 3.1416 3.1032 0.8185]

% Picking up the object from the table while maintaining constant rotation.
% Define the trajectory of the end-effector's origin.
o = transpose([linspace(10, 30, 100)
    linspace(23, 30, 100) 
    linspace(15, 100, 100)]);

% Constant rotation of 45 degrees about z axis
R = [cos(pi/4) -sin(pi/4) 0
    sin(pi/4) cos(pi/4) 0
    0 0 1];

% Matrix to store computed joint angles
q_inverse = zeros([100,6]);

% Loop through all 100 points to compute inverse kinematics
for i = 1:100
    H = eye(4);
    H(1:3,1:3) = R;
    H(1:3, 4) = transpose(o(i,:));
    q_inverse(i,:) = inverse(H, myrobot);
end

% Plot the trajectory
plot3(o(:,1), o(:,2), o(:,3), 'r')
hold on
%Visualize the robot
plot(myrobot, q_inverse)
