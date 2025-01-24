function [robot] = mykuka_search(delta)
	% Generates a KUKA robot model with adjusted DH parameters.

	% Define the original DH parameters of the KUKA robot
    DH_kuka = [25 pi/2 400 0
                315 0 0 0
                35 pi/2 0 0
                0 -pi/2 365 0
                0 pi/2 0 0
                -296.23+delta(1) 0 161.44+delta(2) 0];

    % Create the robot model using the updated DH parameters
    robot = mykuka(DH_kuka);
end

