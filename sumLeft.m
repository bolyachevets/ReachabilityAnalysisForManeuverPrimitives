% compute a lower bound on zonotope directions

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

