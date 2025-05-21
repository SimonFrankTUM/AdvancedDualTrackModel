function A_x = f_xrot(ang)
% Rotation matrix for angle ang about X axis
A_x = [ 1   0           0;
        0   cos(ang)   -sin(ang);
        0   sin(ang)    cos(ang) ];
end