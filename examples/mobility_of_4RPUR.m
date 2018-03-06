% Step 1: fill 'mechanism' array

mechanism_height = 1;
middle_height = 0.5;

middle_scale = 2;
platform_scale = 1.5;
base_shape = [[3; 2], [-2; 1], [-1; -1], [1; -1]];

base_coordinates = [base_shape; zeros(1, 4)];
middle_joint_coordinates = [middle_scale*base_shape; middle_height*ones(1, 4)];
platform_coordinates = [platform_scale*base_shape; mechanism_height*ones(1, 4)];

axis1 = [1; 2; 0];
axis2 = [3; -1; 0];
joint_axes = [axis1, axis2, axis1, axis2];

vertical_axis = [0; 0; 1];

for i = 4:-1:1
    mechanism(i) = struct('matrix', ...
        [mmc_revolute_joint(joint_axes(:, i), base_coordinates(:, i)), ...
         mmc_prismatic_joint(middle_joint_coordinates(:, i) - base_coordinates(:, i)), ...
         mmc_universal_joint(cross(joint_axes(:, i), vertical_axis), middle_joint_coordinates(:, i)), ...
         mmc_revolute_joint(vertical_axis, platform_coordinates(:, i))]);
    mechanism_fixed(i) = struct('matrix', ...
         [mmc_revolute_joint(joint_axes(:, i), base_coordinates(:, i)), ...
          ... %mmc_prismatic_joint(middle_joint_coordinates(:, i) - base_coordinates(:, i)), ...
          mmc_universal_joint(cross(joint_axes(:, i), vertical_axis), middle_joint_coordinates(:, i)), ...
          mmc_revolute_joint(vertical_axis, platform_coordinates(:, i))]);
end

% Step 2: calculate everything
[F, delta, R, ~] = mmc_calculate_mobility(mechanism);
fprintf('Free:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);

[F, delta, R, ~] = mmc_calculate_mobility(mechanism_fixed);
fprintf('Fixed:\n');
fprintf('F = %i, delta = %i, R = %i\n', F, delta, R);
