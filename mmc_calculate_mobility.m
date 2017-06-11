function [F, delta, R, LEGS] = mmc_calculate_mobility(MECHANISM)

% Step 1: compute screws, degrees of freedom and groups dimensions

N = length(MECHANISM); % number of legs
R = 0; % excess number of freedom of the mechanism

for i = N:-1:1
    LEGS(i) = struct('f', size(MECHANISM(i).matrix, 2), ... % degrees of freedom of a leg
                     'Ck', 0, ... % dimension of the kinematic group of a leg
                     'cf', 6, ... % dimension of the force group of a leg
                     'R', 0); % excess degrees of freedom of a leg
    
    LEGS(i).Ck = rank(MECHANISM(i).matrix);
    LEGS(i).cf = 6 - LEGS(i).Ck;
    LEGS(i).R = LEGS(i).f - LEGS(i).Ck;
    
    R = R + LEGS(i).R;
end

% Step 2: compute mechanism's overall groups dimensions

Ck = 0; % dimension of the kinematic group of the mechanism

for q = 1:(2^N-1)
    b = de2bi(q, N);
    s = sum(b);
    
    combination_matrix = [];
    for i = 1:N
        if b(i) == 1
            combination_matrix = [combination_matrix MECHANISM(i).matrix];
        end
    end
    
    Ck = Ck + (-1)^(s-1) * rank(combination_matrix);
end

cf = 6 - Ck; % dimension of the force group of the mechanism

% Step 3: compute mechanism mobility and passive pairs count 

delta = -cf; % number of passive pairs in the mechanism
for i = 1:N
    delta = delta + LEGS(i).cf;
end

F = Ck + R; % degree of mobility of the mechanism

end
