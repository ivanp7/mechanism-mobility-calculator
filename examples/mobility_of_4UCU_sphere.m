% Step 1: fill 'mechanism' array

tetrahedron = sqrt(8/3) * ...
              [[ sqrt(3)/3;    0; -1/4 * sqrt(6)/3], ...
               [-sqrt(3)/6;  1/2; -1/4 * sqrt(6)/3], ...
               [-sqrt(3)/6; -1/2; -1/4 * sqrt(6)/3], ...
               [         0;    0;  3/4 * sqrt(6)/3]];

sphere_radius = 1;
end_effector_radius = 0.25;

leg_inner_end_coordinates = end_effector_radius * tetrahedron;
leg_outer_end_coordinates = sphere_radius * tetrahedron;

leg_axes = leg_outer_end_coordinates - leg_inner_end_coordinates;

for i = 4:-1:1
    mechanism(i) = struct('matrix', ...
        [mmc_universal_joint(leg_axes(:, i), leg_inner_end_coordinates(:, i)), ...
         mmc_cylindrical_joint(leg_axes(:, i), leg_outer_end_coordinates(:, i)), ...
         mmc_universal_joint(leg_axes(:, i), leg_outer_end_coordinates(:, i))]);
    mechanism_fixed(i) = struct('matrix', ...
        [mmc_universal_joint(leg_axes(:, i), leg_inner_end_coordinates(:, i)), ...
         mmc_revolute_joint(leg_axes(:, i), leg_outer_end_coordinates(:, i)), ...
         mmc_universal_joint(leg_axes(:, i), leg_outer_end_coordinates(:, i))]);
end

% Step 2: calculate everything
[F, delta, R, ~] = mmc_calculate_mobility(mechanism);
fprintf('Free:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);

[F, delta, R, ~] = mmc_calculate_mobility(mechanism_fixed);
fprintf('Fixed:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);
