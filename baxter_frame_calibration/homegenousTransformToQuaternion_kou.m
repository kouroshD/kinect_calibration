function [t ,q] = homegenousTransformToQuaternion_kou( T )

t=T(1:3,4);

quat = rotm2quat(T(1:3,1:3)); % [w x y z]
q=[quat(2) quat(3) quat(4) quat(1)];% [x y z w]

end