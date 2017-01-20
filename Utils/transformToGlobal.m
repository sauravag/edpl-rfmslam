function p = transformToGlobal(p, b)

% rotate
rot = [cos(b(3)) -sin(b(3)); sin(b(3)) cos(b(3))];
p(1:2,:) = rot*p(1:2,:);

% translate
p(1,:) = p(1,:) + b(1);
p(2,:) = p(2,:) + b(2);

% if p is a pose and not a point
if size(p,1)==3
    p(3,:) = obj.pi_to_pi(p(3,:) + b(3));
end
end