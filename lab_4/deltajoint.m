function q = deltajoint(delta)
    % DELTAJOINT Computes the calibration cost for a KUKA robot model.

    % Update the KUKA robot model with the given DH parameter perturbations
    kuka = mykuka_search(delta);

    %-------------------------- Calibration ----------------------------%
    % Fill in values Xi and Qi for i = {1, 2, 3}. Xi are 3 by 1
    % column vectors, while Qi are 1 by 6 row vectors.
    
    %-------------------- Measured Positions and Joint Angles --------------------%
    % Measured end-effector positions (Xi) in millimeters
%    X1 = [676.38 0.8 28.24];
%    X2 = [415.94 -533.37 28.24];
%    X3 = [499.21 456.29 28.12];
    X1 = [759.49 -2.31 30.27];
    X2 = [418.81 -633.58 30.27];
    X3 = [544.62 529.36 30.27];

    % Corresponding joint configurations (Qi) in radians
    Q1 = [-0.0007    0.4885   -0.0471   -0.0009    1.5680    0.0003];
    Q2 = [-0.9844    0.4885   -0.0471   -0.0009    1.5680    0.0003];
    Q3 = [0.7735    0.4885   -0.0471   -0.0009    1.5680    0.0003];
%    Q1 = [0.0042 0.6884 -0.1243 0.0103 1.0968 0];
%    Q2 = [-0.9055 0.6884 -0.1243 0.0103 1.0968 0];
%    Q3 = [0.7435 0.6882 -0.1243 0.0103 1.0968 0];
    %-------------------------------------------------------------------%

    % Compute forward kinematics for each joint configuration
    H1=forward_kuka(Q1, kuka);
    H2=forward_kuka(Q2, kuka);
    H3=forward_kuka(Q3, kuka);
    
    % Calculate the total positional discrepancy
    q=norm(H1(1:3,4)-X1)+norm(H2(1:3,4)-X2)+norm(H3(1:3,4)-X3);
end