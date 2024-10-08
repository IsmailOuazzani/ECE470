function [H] = forward_to_link(joint,myrobot, link)
    % compute the forward kinematic transform up to the link specified.

    H = eye(4);
    for i = 1:link
        alpha = myrobot.alpha(i);
        a = myrobot.a(i);
        d = myrobot.d(i);
        theta = joint(i);
        % formula derived in class notes
        H = H * [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
            sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
            0 sin(alpha) cos(alpha) d
            0 0 0 1];
    end
end