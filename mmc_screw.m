function [screw] = mmc_screw(axis, point, parameter)

narginchk(1, 3);

if nargin == 1 % prismatic joint
    if isequal(size(axis), [3 1])
        screw = [[0; 0; 0]; axis];
    else
        error('mmc:mmc_screw', 'Axis vector must be a column of 3 elements.');
    end
else % revolute or screw joint
    p = 0;
    if nargin == 3
        p = parameter;
    end
    
    if isequal(size(axis), [3 1]) && isequal(size(point), [3 1])
        screw = [axis; (cross(axis, point) + p*axis)];
    else
        error('mmc:mmc_screw', 'Axis and point vectors must be columns of 3 elements.');
    end
end

end
