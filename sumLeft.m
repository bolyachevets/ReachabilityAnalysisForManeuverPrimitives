% checks if a set in zonotope representation
% satisfies constraints in interval representation

function sumL = sumLeft(Z)

Gu = get(Z, 'Z');
% generator matrix
Gu = Gu(:, 2:length(Gu));
nrOfGenerators = length(Gu(1,:));

sumL = center(Z);

for i=1:nrOfGenerators
    sumL = sumL - abs(Gu(:,i));
end 

end

