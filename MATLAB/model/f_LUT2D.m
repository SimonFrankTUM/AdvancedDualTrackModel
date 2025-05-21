function vq = f_LUT2D(x,y,V,xq,yq)
% Bilinear interpolation for vq(xq,yq) between values v(x,y) without extrapolation

if isscalar(V)
    vq = V;
    return
end

if xq<x(1) || xq>x(end) || yq<y(1) || yq>y(end)
    error('2D-LUT query point out of bounds.');
end

% Edge coordinates
xIdx1 = f_findIdx1(x,xq);
yIdx1 = f_findIdx1(y,yq);

xqOnXGrid = xq == x(xIdx1);
yqOnYGrid = yq == y(yIdx1);

% Interpolation functions
if xqOnXGrid && ~yqOnYGrid
    vq = f_LUT1D(y,V(xIdx1,:),yq,false);
elseif ~xqOnXGrid && yqOnYGrid
    vq = f_LUT1D(x,V(:,yIdx1),xq,false);
elseif xqOnXGrid && yqOnYGrid
    vq = V(xIdx1,yIdx1);
else
    xIdx2 = xIdx1 + 1;
    yIdx2 = yIdx1 + 1;
    x1 = x(xIdx1);
    x2 = x(xIdx2);
    y1 = y(yIdx1);
    y2 = y(yIdx2);

    vq = 1/((x2-x1)*(y2-y1)) * [x2-xq xq-x1] * ...
        [V(xIdx1,yIdx1) V(xIdx1,yIdx2); V(xIdx2,yIdx1) V(xIdx2,yIdx2)] * ...
        [y2-yq; yq-y1];
end
end