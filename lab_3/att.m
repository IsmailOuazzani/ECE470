function [tau] = att(q,q2, myrobot)
    % q: column vector of actual joint angles
    % q2: column vector of target joint angles
    % myrobot: robot structure

    C = ones(6,1);

    tau = zeros(6,1);

    for i = 1:6
        J_o_i = zeros(3,6);
        c_i = C(i);

        H_i = forward_to_link(q, myrobot, i);
        H_i_target = forward_to_link(q2, myrobot, i);

        % Computing attractive force
        o_i = H_i(1:3,4);
        o_i_target = H_i_target(1:3,4);
        F_att_i = -c_i * (o_i - o_i_target);

        % Computing jacobian
        z_j_prev = [0;0;1];
        o_j_prev = [0;0;0];
        for j = 1:i
            H_j = forward_to_link(q, myrobot, j);
            J_o_i(:,j) = cross(z_j_prev, (o_i_target - o_j_prev));

            z_j_prev = H_j(1:3,3);
            o_j_prev = H_j(1:3,4);
        end
        tau = tau + transpose(J_o_i) *  F_att_i;
    end

    tau = transpose(tau) / norm(tau);
end

