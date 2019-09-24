function T = quaternionToHomogenuesTrasnformation_kou( t, q )
% t=[x ;y ;z]
% q=[x y z w]

quat=[q(4) q(1) q(2) q(3)];% [w x y z]

R= quat2rotm(quat);

T=[R t; 0 0 0 1];

end