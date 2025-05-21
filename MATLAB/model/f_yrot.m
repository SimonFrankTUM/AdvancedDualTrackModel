function A_y = f_yrot(ang)
% Rotation matrix for angle ang about Y axis
A_y = [ cos(ang)    0   sin(ang);
        0           1   0;
       -sin(ang)    0   cos(ang) ];
end