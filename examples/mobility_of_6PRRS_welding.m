% Step 1: fill 'mechanism' array

end_effector_size = 0.25;

body = ...
    [[-1; -1; 0.1], ...
     [-1; -1; -0.1], ...
     [0.1; 1; 1], ...
     [-0.1; 1; 1], ...
     [1; 0.1; -1], ...
     [1; -0.1; -1]];

axes = ...
    [[0; 0; 1], ...
     [0; 0; 1], ...
     [1; 0; 0], ...
     [1; 0; 0], ...
     [0; 1; 0], ...
     [0; 1; 0]];
 
endeff = end_effector_size * ...
    [[-1; -1;  0], ...
     [1; -1;  0], ...
     [0; 1; 1], ...
     [0; -1; 1], ...
     [1; 0; -1], ...
     [1; 0; 1]];

knee = ...
    [[0; -1; 0.1], ...
     [0; -1; -0.1], ...
     [0.1; 0; 1], ...
     [-0.1; 0; 1], ...
     [1; 0.1; 0], ...
     [1; -0.1; 0]];

mechanism(1) = struct('matrix', ...
    [mmc_prismatic_joint(axes(:, 1)), ...
     mmc_revolute_joint(body(:, 1), axes(:, 1)), ...
     mmc_revolute_joint(knee(:, 1), axes(:, 1)), ...
     mmc_spherical_joint(endeff(:, 1))]);
mechanism(2) = struct('matrix', ...
    [mmc_prismatic_joint(axes(:, 2)), ...
     mmc_revolute_joint(body(:, 2), axes(:, 2)), ...
     mmc_revolute_joint(knee(:, 2), axes(:, 2)), ...
     mmc_spherical_joint(endeff(:, 2))]);
mechanism(3) = struct('matrix', ...
    [mmc_prismatic_joint(axes(:, 3)), ...
     mmc_revolute_joint(body(:, 3), axes(:, 3)), ...
     mmc_revolute_joint(knee(:, 3), axes(:, 3)), ...
     mmc_spherical_joint(endeff(:, 3))]);
mechanism(4) = struct('matrix', ...
    [mmc_prismatic_joint(axes(:, 4)), ...
     mmc_revolute_joint(body(:, 4), axes(:, 4)), ...
     mmc_revolute_joint(knee(:, 4), axes(:, 4)), ...
     mmc_spherical_joint(endeff(:, 4))]);
mechanism(5) = struct('matrix', ...
    [mmc_prismatic_joint(axes(:, 5)), ...
     mmc_revolute_joint(body(:, 5), axes(:, 5)), ...
     mmc_revolute_joint(knee(:, 5), axes(:, 5)), ...
     mmc_spherical_joint(endeff(:, 5))]);
mechanism(6) = struct('matrix', ...
    [mmc_prismatic_joint(axes(:, 6)), ...
     mmc_revolute_joint(body(:, 6), axes(:, 6)), ...
     mmc_revolute_joint(knee(:, 6), axes(:, 6)), ...
     mmc_spherical_joint(endeff(:, 6))]);

mechanism_fixed(1) = struct('matrix', ...
    [mmc_revolute_joint(body(:, 1), axes(:, 1)), ...
     mmc_revolute_joint(knee(:, 1), axes(:, 1)), ...
     mmc_spherical_joint(endeff(:, 1))]);
mechanism_fixed(2) = struct('matrix', ...
    [mmc_revolute_joint(body(:, 2), axes(:, 2)), ...
     mmc_revolute_joint(knee(:, 2), axes(:, 2)), ...
     mmc_spherical_joint(endeff(:, 2))]);
mechanism_fixed(3) = struct('matrix', ...
    [mmc_revolute_joint(body(:, 3), axes(:, 3)), ...
     mmc_revolute_joint(knee(:, 3), axes(:, 3)), ...
     mmc_spherical_joint(endeff(:, 3))]);
mechanism_fixed(4) = struct('matrix', ...
    [mmc_revolute_joint(body(:, 4), axes(:, 4)), ...
     mmc_revolute_joint(knee(:, 4), axes(:, 4)), ...
     mmc_spherical_joint(endeff(:, 4))]);
mechanism_fixed(5) = struct('matrix', ...
    [mmc_revolute_joint(body(:, 5), axes(:, 5)), ...
     mmc_revolute_joint(knee(:, 5), axes(:, 5)), ...
     mmc_spherical_joint(endeff(:, 5))]);
mechanism_fixed(6) = struct('matrix', ...
    [mmc_revolute_joint(body(:, 6), axes(:, 6)), ...
     mmc_revolute_joint(knee(:, 6), axes(:, 6)), ...
     mmc_spherical_joint(endeff(:, 6))]);

% Step 2: calculate everything
[F, delta, R, ~] = mmc_calculate_mobility(mechanism);
fprintf('Free:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);

[F, delta, R, ~] = mmc_calculate_mobility(mechanism_fixed);
fprintf('Fixed:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);