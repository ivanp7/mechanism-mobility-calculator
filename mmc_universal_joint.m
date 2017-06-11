function [screw_matrix] = mmc_universal_joint(missing_axis, origin)

if ~isequal(size(missing_axis), [3 1])
    error('mmc:mmc_universal_joint', 'Axis vector must be a column of 3 elements.');
end

missing_axis_norm = sqrt(sum(missing_axis.^2));
if missing_axis_norm < 1e-6
    error('mmc:mmc_universal_joint', 'Axis vector norm is too small.');
end
missing_axis = missing_axis / missing_axis_norm;

axis1_norm = 0;
while axis1_norm < 1e-6
    random_vector = rand(3, 1);
    axis1 = cross(missing_axis, random_vector);
    axis1_norm = sqrt(sum(axis1.^2));
end

axis1 = axis1 / axis1_norm;
axis2 = cross(missing_axis, axis1);

screw_matrix = [mmc_revolute_joint(axis1, origin), ...
                mmc_revolute_joint(axis2, origin)];

end
