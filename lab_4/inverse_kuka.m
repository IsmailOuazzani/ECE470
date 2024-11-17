function [q] = inverse_kuka(H, myrobot)
    % Computes inverse kinematics through kinematic decoupling method.
    % Closed form solution was derived in the prelab.
    
    % Extract relevant DH parameters
    a_1 = myrobot.a(1);
    a_2 = myrobot.a(2);
    a_3 = myrobot.a(3);
    a_6 = myrobot.a(6);
    d_1 = myrobot.d(1);
    d_4 = myrobot.d(4);
    d_4_adjusted = sqrt(a_3^2 + (d_4)^2);
    d_6 = myrobot.d(6);
    
    % Comput wrist location
    desired_end_effector_coordinates = H(1:3, 4) - H(1:3,1:3) * [a_6; 0; d_6];
    x_c = desired_end_effector_coordinates(1);
    y_c = desired_end_effector_coordinates(2);
    z_c = desired_end_effector_coordinates(3);
    
    % Compute auxiliary variables for calculations
    r_1 = sqrt(x_c^2 + y_c^2) - a_1;
    s = z_c - d_1;
    D = (r_1^2 + s^2 - a_2^2 - d_4_adjusted^2)/(2*a_2*d_4_adjusted);
    % Compute the joint angles for the first three joints
    theta_1 = atan2(y_c, x_c);
    theta_3 = atan2(D, sqrt(1-D^2)) - atan2(a_3, d_4);
    l = d_4_adjusted*sin(theta_3-pi/2 + atan2(a_3, d_4));
    m = d_4_adjusted*cos(theta_3-pi/2 + atan2(a_3, d_4));
    theta_2 = atan2(s,r_1) - atan2(l, a_2+m);

    % Compute the transformation matrix for the first three joints
    H_3_0 = forward_to_link([theta_1 theta_2 theta_3], myrobot, 3);

    % Compute the relative transformation from the wrist to the end-effector
    H_6_3 = transpose(H_3_0) * H;

    % Compute the remaining joint angles from rotation matrices 
    theta_4 = atan2(H_6_3(2,3),H_6_3(1,3));
    theta_5 = atan2(sqrt(1-H_6_3(3,3)^2), H_6_3(3,3));
    theta_6 = atan2(H_6_3(3,2), -H_6_3(3,1));

    % Output joint angles as column vector
    q = [theta_1
        theta_2
        theta_3
        theta_4
        theta_5
        theta_6];

end