% DH Table Initialization for Puma 560
DH_kuka = [25 pi/2 400 0
    315 0 0 0
    35 pi/2 0 0
    0 -pi/2 365 0
    0 pi/2 0 0
    -296.23 0 161.44 0];

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

tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], kuka_forces, prepobs{1}) % 0.1795    0.9540    0.2353   -0.0344   -0.0344    0.0000
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
qref=motionplan(q1, q2,0,10, kuka_forces, prepobs, 0.01)
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(kuka,q);
hold off