function r_tilde = f_crossprod(r)
% Prepare skew symmetric cross product matrix from vector r
r_tilde = [ 0      -r(3)    r(2);
            r(3)    0      -r(1);
           -r(2)    r(1)    0 ];
end