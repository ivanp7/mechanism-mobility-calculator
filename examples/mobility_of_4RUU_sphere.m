% Step 1: fill 'mechanism' array

tetrahedron = sqrt(2/3) * ...
              [[-1;  0; -sqrt(1/2)], ...
               [ 1;  0; -sqrt(1/2)], ...
               [ 0; -1;  sqrt(1/2)], ...
               [ 0;  1;  sqrt(1/2)]];

sphere_radius = 1;
end_effector_radius = 0.25;

leg_inner_end_coordinates = end_effector_radius * tetrahedron;
leg_outer_end_coordinates = sphere_radius * tetrahedron;

leg_axes = leg_outer_end_coordinates - leg_inner_end_coordinates;
leg_middle_coordinates = leg_inner_end_coordinates + leg_axes/2;

rev_in_axis = [[0; 1; 0], [0; 1; 0], [1; 0; 0], [1; 0; 0]];
rev_out_axis = [[0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1]];
uni_axis = [[0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1]]; %rev_out_axis%;%cross(rev_in_axis, rev_out_axis);

leg_knee_coordinates = leg_middle_coordinates + (sphere_radius - end_effector_radius)/2 * uni_axis;

for i = 4:-1:1
    mechanism(i) = struct('matrix', ...
        [mmc_revolute_joint(rev_in_axis(:, i), leg_inner_end_coordinates(:, i)), ...
         mmc_universal_joint(uni_axis(:, i), leg_knee_coordinates(:, i)), ...
         mmc_universal_joint(rev_out_axis(:, i), leg_outer_end_coordinates(:, i))]);
    mechanism_fixed(i) = struct('matrix', ...
        [mmc_universal_joint(uni_axis(:, i), leg_knee_coordinates(:, i)), ...
         mmc_universal_joint(rev_out_axis(:, i), leg_outer_end_coordinates(:, i))]);
end

% Step 2: calculate everything
[F, delta, R, ~] = mmc_calculate_mobility(mechanism);
fprintf('Free:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);

[F, delta, R, ~] = mmc_calculate_mobility(mechanism_fixed);
fprintf('Fixed:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);
