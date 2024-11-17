function [tau] = rep(q, myrobot, obs)
    % Repulsive potential

    % Params:
    % q: column vector of actual joint angles
    % myrobot: robot structure
    % obs is an obstacle structure

    % Returns:
    % tau: column vector of repulsive torques

    tau = zeros(6,1);
    eta = 1;
    F_rep = zeros(3,6);

    for i = 1:6
        J_o_i = zeros(3,6);
        H_i = forward_to_link(q, myrobot, i);

        % Computing repulsive force
        o_i = H_i(1:3,4);
        if obs.type == 'cyl' % of finite height
            d_rad = max(0, norm(o_i(1:2) - obs.c) - obs.R);
            d_z = max(0, o_i(3) - obs.h);
            o_i_b_norm = sqrt(d_rad^2 + d_z^2);
            o_i_b = [0;0;d_z];
            o_i_b(1:2) = (o_i(1:2) - obs.c) * (1-obs.R/(d_rad + obs.R));
        elseif obs.type == 'sph'
            o_i_b_norm = norm(o_i - obs.c) - obs.R;
            o_i_b = (o_i - obs.c) * (1-obs.R/(o_i_b_norm+obs.R)); 
        elseif obs.type == 'pln' % plane
            o_i_b_norm = (o_i(3) - obs.z);
            o_i_b = [0;0;o_i_b_norm];
        end
        
        if o_i_b_norm <= obs.rho0 
            F_rep_i = eta*(1/o_i_b_norm - 1/obs.rho0)*(o_i_b/(o_i_b_norm^3));
        else
            F_rep_i = [0;0;0];
        end
        F_rep(:,i) = F_rep_i;

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

    % Uncomment to view the repulsive forces:
    % F_rep
end

