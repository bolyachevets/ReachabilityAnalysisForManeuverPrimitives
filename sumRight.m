% compute an upper bound on zonotope directions

function sumR = sumRight(Z)

Gu = get(Z, 'Z');
% generator matrix
Gu = Gu(:, 2:length(Gu));
nrOfGenerators = length(Gu(1,:));

sumR = center(Z);

for i=1:nrOfGenerators
    sumR = sumR + abs(Gu(:,i));
end 

end

