function [qref] = motionplan(q0, q2, t1, t2, myrobot, obs, tol)
    % Gradient descent algorithm for motion planning of robot manipulator
  
    % Params:
    % q0 is a column vector with actual joint angles
    % q2 is a column vector with the final joint angles
    % t1 is start time
    % t2 is the finish time
    % obs contains the list of obstacles
    % tol is the tolerance for algorithm termination

    % Returns:
    % qref is a piecewise cubic polynomial trajectory


    alpha_att = 0.01;
    alpha_rep = 0.01;

    q = transpose(q0);
    iterations = 0;
    max_iter = 5000;
    while norm(q(end,1:5)-q2(1:5)')>tol && (iterations < max_iter)
        q_cur = q(end, 1:6);
        q(end+1,1:6) =  q_cur + alpha_att * att(q_cur', q2, myrobot);
        
        for j = 1:size(obs,2)
            q(end, 1:6) = q(end, 1:6) + alpha_rep * rep(q_cur', myrobot, obs{j});
        end
        
        if mod(iterations, 100) == 0 
            fprintf('Iter: %f Error: %f \n',iterations, norm(q(end,1:5)-q2(1:5)'));
        end

        iterations = iterations + 1;
    end

    q(:,6) = linspace(q0(6),q2(6),size(q,1));

    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q'); % defines a spline object with interpolation
    % times in t and interpolation values the columns of q
end

