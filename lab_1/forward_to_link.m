function [H] = forward_to_link(joint,myrobot, link)
    % Computes the forward kinematic transform up to the link number specified.

    % Initialize transformation matrix to identify matrix
    H = eye(4);

    % Loop through each link
    for i = 1:link
        % Extract DH parameters for link i
        alpha = myrobot.alpha(i);
        a = myrobot.a(i);
        d = myrobot.d(i);
        theta = joint(i);

        % Compute transformation matrix
        % Formula derived in class notes
        H = H * [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
            sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
            0 sin(alpha) cos(alpha) d
            0 0 0 1];
    end
end