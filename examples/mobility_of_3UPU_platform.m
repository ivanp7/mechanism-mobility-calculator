% Step 1: fill 'mechanism' array

triangle = [[   1;          0; 0], ...
            [-1/2; -sqrt(3)/2; 0], ...
            [-1/2;  sqrt(3)/2; 0]];

base_radius = 1;
platform_radius = 0.5;
platform_height = 1;

leg_bottom_end_coordinates = base_radius * triangle;
leg_top_end_coordinates = platform_radius * triangle + platform_height * [0; 0; 1];

leg_axes = leg_top_end_coordinates - leg_bottom_end_coordinates;

mechanism(1) = struct('matrix', ...
    [mmc_universal_joint(leg_axes(:, 1), leg_bottom_end_coordinates(:, 1)), ...
     mmc_prismatic_joint(leg_axes(:, 1)), ...
     mmc_universal_joint(leg_axes(:, 1), leg_top_end_coordinates(:, 1))]);
mechanism(2) = struct('matrix', ...
    [mmc_universal_joint(leg_axes(:, 2), leg_bottom_end_coordinates(:, 2)), ...
     mmc_prismatic_joint(leg_axes(:, 2)), ...
     mmc_universal_joint(leg_axes(:, 2), leg_top_end_coordinates(:, 2))]);
mechanism(3) = struct('matrix', ...
    [mmc_universal_joint(leg_axes(:, 3), leg_bottom_end_coordinates(:, 3)), ...
     mmc_prismatic_joint(leg_axes(:, 3)), ...
     mmc_universal_joint(leg_axes(:, 3), leg_top_end_coordinates(:, 3))]);

mechanism_fixed(1) = struct('matrix', ...
    [mmc_universal_joint(leg_axes(:, 1), leg_bottom_end_coordinates(:, 1)), ...
     mmc_universal_joint(leg_axes(:, 1), leg_top_end_coordinates(:, 1))]);
mechanism_fixed(2) = struct('matrix', ...
    [mmc_universal_joint(leg_axes(:, 2), leg_bottom_end_coordinates(:, 2)), ...
     mmc_universal_joint(leg_axes(:, 2), leg_top_end_coordinates(:, 2))]);
mechanism_fixed(3) = struct('matrix', ...
    [mmc_universal_joint(leg_axes(:, 3), leg_bottom_end_coordinates(:, 3)), ...
     mmc_universal_joint(leg_axes(:, 3), leg_top_end_coordinates(:, 3))]);

% Step 2: calculate everything
[F, delta, R, ~] = mmc_calculate_mobility(mechanism);
fprintf('Free:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);

[F, delta, R, ~] = mmc_calculate_mobility(mechanism_fixed);
fprintf('Fixed:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);