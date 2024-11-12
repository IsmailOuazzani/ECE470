function [tau] = rep(q, myrobot, obs)
    % q: column vector of actual joint angles
    % myrobot: robot structure
    % obs is an obstacle structure

    tau = zeros(6,1);
    eta = 1;

    for i = 1:6
        J_o_i = zeros(3,6);
        H_i = forward_to_link(q, myrobot, i);

        % Computing repulsive force, assuming obstacles are either of type
        % cylinder (cyl) or sphere (sph)
        o_i = H_i(1:3,4);
        if obs.type == 'cyl'
            o_i_b_norm = norm(o_i(1:2) - obs.c) - obs.R;
            o_i_b = (o_i(1:2) - obs.c) * (1-obs.R/o_i_b_norm);
            o_i_b(3) = 0;
        else
            o_i_b_norm = norm(o_i - obs.c) - obs.R;
            o_i_b = (o_i - obs.c) * (1-obs.R/(o_i_b_norm+obs.R)); 
        end
        
        if o_i_b_norm <= obs.rho0
            F_rep_i = eta*(1/o_i_b_norm - 1/obs.rho0)*(o_i_b/(o_i_b_norm^3));
        else
            F_rep_i = [0;0;0];
        end

        % Computing jacobian
        z_j_prev = [0;0;1];
        o_j_prev = [0;0;0];
        for j = 1:i
            H_j = forward_to_link(q, myrobot, j);
            J_o_i(:,j) = cross(z_j_prev, (o_i - o_j_prev));

            z_j_prev = H_j(1:3,3);
            o_j_prev = H_j(1:3,4);
        end
        tau = tau + transpose(J_o_i) *  F_rep_i;
    end

    tau = tau';
    if norm(tau)> 0
        tau = tau / norm(tau);
    end
end

