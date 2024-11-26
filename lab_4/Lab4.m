close all

% DH Table Initialization for PKuka robot
DH_kuka = [25 pi/2 400 0
    315 0 0 0
    35 pi/2 0 0
    0 -pi/2 365 0
    0 pi/2 0 0
    -156 0 161.44 0];

DH_forces = [25 pi/2 400 0
    315 0 0 0
    35 pi/2 0 0
    0 -pi/2 365 0
    0 pi/2 0 0
    0 0 161.44 0];

kuka = mykuka(DH_kuka);
kuka_forces = mykuka(DH_forces);

%% 3.0 Obstacle Calculations Prep
setupobstacle_lab4prep % load obstacles into workspace

tau=rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], kuka_forces, prepobs{1}) % 0.1795    0.9540    0.2353   -0.0344   -0.0344    0.0000
% We obtained the following values F_rep =
  %  1.0e-06 *

  %  -0.1269    0.0059    0.0073    0.0168    0.0168    0.0189
  %   0.0043    0.0096    0.0092    0.0096    0.0096    0.0018
  %   0.1005    0.1037    0.0615         0         0         0


% Testing motionplan
p1 = [620 375 50];
p2 = [620 -375 50];
R=[0 0 1;0 -1 0;1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
q2 = inverse_kuka(H2, kuka);
q1 = inverse_kuka(H1, kuka);

% Plot trajectory
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(prepobs);
qref=motionplan(q1, q2,0,10, kuka_forces, prepobs, 0.01);
t=linspace(0,10,100);
q=ppval(qref,t)';
plot(kuka,q,'notiles');
hold off

%%  4.1 Initial Motion Planning in Simulation
clf() % Get rid of the obstacles from the previous section
setupobstacle

z_grid = 45; % mm
p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];

desired_points = [p0;p1;p2;p3];

qref = multi_pt(desired_points, kuka, kuka_forces, obs);


% Plot trajectory
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qref,'notiles');
hold off

%% Calibrating the position of the cylinders
% The purpose of this section was to place the obstacles at known locations. 
% To do so, we make the robot go to a location of our choosing and mark it with a piece of tape.
% The tape denotes the center of the obstacle. We leave this section commented out as it contains 
% commands to the physical robot.

% p_cyl_1 = [620, 0, 50];
% desired_pts = [p_cyl_1; p_cyl_1];
% qref_cyl_1 = multi_pt(desired_pts, kuka, kuka_forces, []);
% perform_traj(qref_cyl_1);

% p_cyl_2 = [620, -440, 50];
% desired_pts = [p_cyl_2; p_cyl_2];
% qref_cyl_2 = multi_pt(desired_pts, kuka, kuka_forces, []);
% perform_traj(qref_cyl_2);

% p_cyl_block = p1;
% p_cyl_block(3) = 50;
% desired_pts = [p_cyl_block; p_cyl_block];
% qref_block = multi_pt(desired_pts, kuka, kuka_forces, []);
% perform_traj(qref_block);

% p_cyl_block_2 = p3;
% p_cyl_block_2(1) = p_cyl_block_2(1)-300;
% p_cyl_block_2(3) = 50;
% desired_pts = [p_cyl_block_2; p_cyl_block_2];
% qref_block_2 = multi_pt(desired_pts, kuka, kuka_forces, []);
% perform_traj(qref_block_2);

% p_cyl_basket = p3;
% p_cyl_basket(3) = 50;
% desired_pts = [p_cyl_basket; p_cyl_basket];
% qref_basket = multi_pt(desired_pts, kuka, kuka_forces, []);
% perform_traj(qref_basket);

%% 4.2 Initial Motion Planning with Kuka
% We leave the code that sends commands to the robot commented out.

% Test trajectory to verify that the robot motion in the physical world
% matches the one in simulation.
desired_points = [p0;p1;p2;p3];
qref = multi_pt(desired_points, kuka, kuka_forces, obs);
% perform_traj(qref);


% To displace the cube, we break down the motion into smaller segments,
% so that we are able to control the gripper in between.
qref0 = multi_pt([p0;p0], kuka, kuka_forces, obs);
Ree = [0 0 1; 0 -1 0; 1 0 0];
H1 = [Ree p1';zeros(1,3) 1];
qref1 = inverse_kuka(H1, kuka);
qref2 = multi_pt([p1;p2;p3], kuka, kuka_forces, obs);
% perform_traj(qref0);
% setGripper(0);
% setAngles(qref1, 0.03);
% setGripper(1);
% perform_traj(qref2);
% setGripper(0);


%% 4.3 Creative Motion Planning with Kuka
% % qref0 = multi_pt([p0;p0], kuka, kuka_forces, obs);
% % Ree = [0 0 1; 0 -1 0; 1 0 0];
% % H1 = [Ree p1';zeros(1,3) 1];
% % qref1 = inverse_kuka(H1, kuka);
% % qref2 = multi_pt([p1;p2;p3], kuka, kuka_forces, obs);
% % p_cyl_block_2_prepare = p_cyl_block_2;
% % p_cyl_block_2_prepare(3) = 70;
% % qref3 = multi_pt([p3;p3;p_cyl_block_2_prepare], kuka, kuka_forces, obs);
% % H4 = [Ree p_cyl_block_2';zeros(1,3) 1];
% % qref4 = inverse_kuka(H4, kuka);
% % qref5 = multi_pt([p_cyl_block_2;p_cyl_block_2;p3], kuka, kuka_forces, obs);

% % qref2 = qref2(101:size(qref2,1),:);
% % qref3 = qref3(101:size(qref3,1),:);
% % qref5 = qref5(101:size(qref5,1),:);


% perform_traj(qref0);
% setGripper(0);
% setAngles(qref1, 0.03);
% setGripper(1);
% perform_traj(qref2);
% setGripper(0);
% perform_traj(qref3);
% setAngles(qref4,0.03);
% setGripper(1);
% perform_traj(qref5);
% setGripper(0);

function [qref] = multi_pt(points, robot_regular, robot_forces, obstacles)
    % Plan a trajectory between multiple points

    % Params:
    % points: a matrix with the desired points
    % robot_regular: robot object with the regular DH table
    % robot_forces: robot object with the DH table for forces
    % obstacles: a cell array with the obstacles

    % Returns:
    % qref: a piecewise cubic polynomial trajectory

    Ree = [0 0 1; 0 -1 0; 1 0 0];
    n_motions = size(points,1);
    qref = zeros(100*(n_motions-1),6);
    q_home = [0 pi/2 0 0 pi/2 0]';
    q_cur = q_home;
    for i = 1:n_motions-1
        disp('Planning trajectory between')
        pi_cur=points(i,:);
        pi_next=points(i+1,:);
        disp(pi_cur)
        disp(pi_next)
        H_next=[Ree pi_next';zeros(1,3) 1];
        q_next = inverse_kuka(H_next, robot_regular);
        qref_i = motionplan(q_cur, q_next, 0, 10, robot_forces, obstacles, 0.01);
        t_i =linspace(0,10,100);
        q_i=ppval(qref_i,t_i)';
        qref((i-1)*100+1:i*100,:)=q_i;
        fprintf('Successfully computed trajectory between points %f and %f \n',i,i+1)
        q_cur = q_next;
    end
end

function [un] = perform_traj(qref)
    % Send commands to the robot to perform the trajectory
  
    % Params:
    % qref: a piecewise cubic polynomial trajectory
    for i = 1:size(qref,1)
        setAngles(qref(i,:), 0.03);
    end
    disp("Finished moving baby")
end
