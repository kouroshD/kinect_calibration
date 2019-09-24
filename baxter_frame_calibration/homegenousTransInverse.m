function inv_T = homegenousTransInverse( T )
R=T(1:3,1:3);
t=T(1:3,4);
inv_T=[R',-R'*t;0 0 0 1];
end
