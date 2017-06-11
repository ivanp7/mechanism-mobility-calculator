function [screw_matrix] = mmc_spherical_joint(origin)

screw_matrix = [mmc_revolute_joint([1; 0; 0], origin), ...
                mmc_revolute_joint([0; 1; 0], origin), ...
                mmc_revolute_joint([0; 0; 1], origin)];

end
