%% Definition of robot structure

% values were computed in prelab
DH_puma560 = [0 pi/2 76 0
    43.23 0 -23.65 0
    0 pi/2 0 0
    0 -pi/2 43.18 0
    0 pi/2 0 0
    0 0 20 0];

myrobot = mypuma560(DH_puma560);

%% 4.2 Plot a sample joint spce trajectory

q = [linspace(0, pi, 200);
    linspace(0, pi/2, 200) 
    linspace(0, pi, 200)
    linspace(pi/4, 3*pi/4, 200)
    linspace(-pi/3, pi/3, 200)
    linspace(0, 2*pi, 200)];
q = transpose(q);


plot(myrobot, q)

%% 4.3 Forward Kinematics

o = zeros([200,3]);
for i = 1:200
    H =  forward(q(i,:), myrobot);
    end_effector_frame = H*ones([4,1]);
    o(i,:) = end_effector_frame(1:3);
end

plot3(o(:,1), o(:,2), o(:,3), 'r')
hold on
plot(myrobot, q)

%% 4.4 Inverse Kinematics

% First we test that our inverse kinematic function is working as expected.
test_H = [cos(pi/4) -sin(pi/4) 0 20
    sin(pi/4) cos(pi/4) 0 23
    0 0 1 15
    0 0 0 1];
inverse(test_H, myrobot) % expected: [ -0.0331 -1.0667 1.0283 3.1416 3.1032 0.8185]

% Picking up the object from the table while maintaining constant rotation.
o = transpose([linspace(10, 30, 100);
    linspace(23, 30, 100) 
    linspace(15, 100, 100)
]);
R = [cos(pi/4) -sin(pi/4) 0
    sin(pi/4) cos(pi/4) 0
    0 0 1];
q_inverse = zeros([100,6]);
for i = 1:100
    H = eye(4);
    H(1:3,1:3) = R;
    H(1:3, 4) = transpose(o(i,:));
    q_inverse(i,:) = inverse(H, myrobot);
end

plot(myrobot, q_inverse)
