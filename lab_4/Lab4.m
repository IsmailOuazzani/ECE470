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

% Load new obstacles
setupobstacle

% Setup points as per lab handout
z_grid = 45; % mm
p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];

% Define home position joint angles
q_home = [0 pi/2 0 0 pi/2 0]';

% Define points in trajectory
desired_points = [p0;p1;p2;p3];

% Spline interpolate using multi_pt function
qref = multi_pt(desired_points, q_home, kuka, kuka_forces, obs);


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
% To do so, we command the robot to go to a location of our choosing and mark it with a piece of tape.
% The tape denotes the center of the obstacle.
% We leave the code that sends commands to the robot commented out.

% Define home position joint angles
q_home = [0 pi/2 0 0 pi/2 0]';

%% Send the robot to the location of the first cylinder obstacle

% Define cylinder 2 position and spline interpolate motion from home to it
p_cyl_1 = [620, 0, 50];
desired_pts = [p_cyl_1];
qref_cyl_1 = multi_pt(desired_pts, q_home, kuka, kuka_forces, []);

% Command to send robot to the first cylinder obstacle (from home)
% perform_traj(qref_cyl_1);

% Plot trajectory
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qref_cyl_1,'notiles');
hold off

%% Send the robot to the location of the second cylinder obstacle

% Define cylinder 2 position and spline interpolate motion from home to it
p_cyl_2 = [620, -440, 50];
desired_pts = [p_cyl_2];
qref_cyl_2 = multi_pt(desired_pts, q_home, kuka, kuka_forces, []);

% Command to send robot to the second cylinder obstacle (from home)
% perform_traj(qref_cyl_2);

% Plot trajectory
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qref_cyl_2,'notiles');
hold off

%% Send the robot to the block pickup location

% Define block pickup location
p_block = p1;
p_block(3) = 50;
desired_pts = [p_block];
qref_block = multi_pt(desired_pts, q_home, kuka, kuka_forces, []);

% Command to send robot to block pickup location (from home)
% perform_traj(qref_block);

% Plot trajectory
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qref_block,'notiles');
hold off

%% Send robot to second block pickup location (for creative motion planning)

% Define a new second block pickup location
p_block_2 = p3;
p_block_2(1) = p_block_2(1)-300;
p_block_2(3) = 50;
desired_pts = [p_block_2];
qref_block_2 = multi_pt(desired_pts, q_home, kuka, kuka_forces, []);

% Command to send robot to second block pickup location (from home)
% perform_traj(qref_block_2);

% Plot trajectory
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qref_block_2,'notiles');
hold off

%% Send Robot to Basket Location

% Define basket position 
p_cyl_basket = p3; 
p_cyl_basket(3) = 50;
desired_pts = [p_cyl_basket];
qref_basket = multi_pt(desired_pts, q_home, kuka, kuka_forces, []);

% Send robot to basket location 
% perform_traj(qref_basket);

% Plot trajectory
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qref_basket,'notiles');
hold off

%% 4.2 Initial Motion Planning with Kuka
% We leave the code that sends commands to the robot commented out.

% Define home position joint angles
q_home = [0 pi/2 0 0 pi/2 0]';

% Test trajectory to verify that the robot motion in the physical world
% matches the one in simulation.
desired_points = [p0;p1;p2;p3];
qref = multi_pt(desired_points, q_home, kuka, kuka_forces, obs);

% Send robot along trajectory
% perform_traj(qref);

%% Send Robot along trajectory with gripping motions
% To displace the cube, we break down the motion into smaller segments,
% so that we are able to control the gripper in between.

% Overall motion:
% 1. Home->p0 : motion plan
% 2. Open Gripper (to be ready to close on block)
% 3. p0->p1 : setAngles (as p1 directly over p0)
% 4. Close Gripper
% 5. p1->p3 : motion plan
% 6. Open Gripper

qref0 = multi_pt([p0], q_home, kuka, kuka_forces, obs);

q0 = qref0(end, :)';
Ree = [0 0 1; 0 -1 0; 1 0 0];
H1 = [Ree p1';zeros(1,3) 1];
q1 = inverse_kuka(H1, kuka);
qref1 = multi_pt([p1], q0, kuka, kuka_forces, obs);

qref2 = multi_pt([p2;p3], q1, kuka, kuka_forces, obs);


% Simulation of trajectory
qreftotal = [qref0;qref1;qref2];
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qreftotal,'notiles');
hold off

% Send robot along entire trajectory
% perform_traj(qref0);
% setGripper(0);
% setAngles(q1, 0.03); % Smoother to just setAngles instead of motion plan
% setGripper(1);
% perform_traj(qref2);
% setGripper(0);


%% 4.3 Creative Motion Planning with Kuka
% We leave the code that sends commands to the robot commented out.
clf() % Get rid of the obstacles from the previous section

% Overall idea:
% Define a second block location and pick up both blocks indidually to put
% them both into basket one at a time

% Overall motion:
% 1. Home->p0 : motion plan
% 2. Open Gripper (to be ready to close on block 1)
% 3. p0->p1 : setAngles (as p1 directly over p0)
% 4. Close Gripper
% 5. p1->p3 : motion plan
% 6. Open Gripper
% 7. p3->p_block_2_prepare : motion plan
% 8. p_block_2_prepare -> p_block_2 : setAngles (as p_block_2_prepare
% directly over p_block_2)
% 9. Close Gripper
% 10. p_block_2-> p3
% 11. Open Gripper

% Load obstacles
setupobstacle

q_home = [0 pi/2 0 0 pi/2 0]';
qref0 = multi_pt([p0], q_home, kuka, kuka_forces, obs);

q0 = qref0(end, :)';
Ree = [0 0 1; 0 -1 0; 1 0 0];
H1 = [Ree p1';zeros(1,3) 1];
q1 = inverse_kuka(H1, kuka);
qref1 = multi_pt([p1], q0, kuka, kuka_forces, obs);

qref2 = multi_pt([p2;p3], q1, kuka, kuka_forces, obs);

q3 = qref2(end, :)';
p_block_2_prepare = p_block_2;
p_block_2_prepare(3) = 70;
qref3 = multi_pt([p_block_2_prepare], q3, kuka, kuka_forces, obs);

q_block_2_prepare = qref3(end, :)';
H2 = [Ree p_block_2';zeros(1,3) 1];
q_block_2 = inverse_kuka(H2, kuka);
qref4 = multi_pt([p_block_2], q_block_2_prepare, kuka, kuka_forces, obs);

q_block_2 = qref4(end,:)';
qref5 = multi_pt([p3], q_block_2, kuka, kuka_forces, obs);

% Simulation of trajectory (without pauses for gripper)
qreftotal = [qref0;qref1;qref2;qref3;qref4;qref5];
hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(obs);
t=linspace(0,10,100);
plot(kuka,qreftotal,'notiles');
hold off



% Send robot along entire trajectory
% perform_traj(qref0);
% setGripper(0);
% setAngles(q1, 0.03); % Smoother to just setAngles instead of motion plan
% setGripper(1);
% perform_traj(qref2);
% setGripper(0);
% perform_traj(qref3);
% setAngles(q_block_2,0.03); % Smoother to just setAngles instead of motion plan
% setGripper(1);
% perform_traj(qref5);
% setGripper(0);

function [qref] = multi_pt(points, q_start, robot_regular, robot_forces, obstacles)
    % Plan a trajectory between multiple points

    % Params:
    % points: a matrix with the desired points
    % robot_regular: robot object with the regular DH table
    % robot_forces: robot object with the DH table for forces
    % obstacles: a cell array with the obstacles

    % Returns:
    % qref: a piecewise cubic polynomial trajectory

    % Define end effector rotation
    Ree = [0 0 1; 0 -1 0; 1 0 0];

    % Identify number of points to iterate through
    n_motions = size(points,1);

    % Initialize joint reference angles matrix for spline
    qref = zeros(100*(n_motions),6);

    % Identify starting joint angles as q_cur
    q_cur = q_start;

    for i = 1:n_motions
        % Find the joint angles given point in trajectory
        pi_next=points(i,:);
        H_next=[Ree pi_next';zeros(1,3) 1];
        q_next = inverse_kuka(H_next, robot_regular);

        % Motion plan between current joint angles and new joint angles
        qref_i = motionplan(q_cur, q_next, 0, 10, robot_forces, obstacles, 0.01);
        t_i =linspace(0,10,100);
        q_i=ppval(qref_i,t_i)';
        qref((i-1)*100+1:i*100,:)=q_i;
        fprintf('Successfully computed trajectory between points %f and %f \n',i,i+1)

        % New joint angle becomes current after spline interpolation
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


function [qref] = simulate_set_angle(last_q, q, n)
    % Simulate the trajectory between two points

    % Params:
    % last_q: the initial point
    % q: the final point
    % n: the number of points in the trajectory

    % Returns:
    % qref: a piecewise cubic polynomial trajectory

    qref = zeros(n,6);
    for i = 1:6
        qref(:,i) = linspace(last_q(i),q(i),n);
    end
end