function [H] = forward_kuka(joint,myrobot)
    % Computes forward kinematics for the entire robot given current joint states.
    H = forward_to_link(joint,myrobot,6);
end