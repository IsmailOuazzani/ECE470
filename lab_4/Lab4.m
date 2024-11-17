% DH Table Initialization for Puma 560
DH_kuka = [25 pi/2 400 0
    315 0 0 0
    35 pi/2 0 0
    0 -pi/2 365 0
    0 pi/2 0 0
    156 0 161.44 0];

DH_forces = [25 pi/2 400 0
    315 0 0 0
    35 pi/2 0 0
    0 -pi/2 365 0
    0 pi/2 0 0
    0 0 161.44 0];

kuka = mykuka(DH_kuka);

kuka_forces = mykuka(DH_forces);

%% 4.0 Obstacle Calculations Prep
setupobstacle_lab4prep % load obstacles into workspace

tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], kuka_forces, prepobs{1})
