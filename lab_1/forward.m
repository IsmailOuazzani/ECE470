function [H] = forward(joint,myrobot)
    % computes forward kinematics for the entire robot given current joint states.
    H = forward_to_link(joint,myrobot,6);
end