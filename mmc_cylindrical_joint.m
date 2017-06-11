function [screw_matrix] = mmc_cylindrical_joint(axis, point)

screw_matrix = [mmc_prismatic_joint(axis), ...
                mmc_revolute_joint(axis, point)];

end
