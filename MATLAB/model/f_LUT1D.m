function vq = f_LUT1D(x,v,xq,extrapolate)
% Linear interpolation for vq(xq) between values v(x)

if isscalar(v)
    vq = v;
    return
end

if xq <= x(1) % xq is start point
    if extrapolate
        % Extrapolation function
        vq = v(1) + (v(2)-v(1))*(xq-x(1))/(x(2)-x(1));
    else
        vq = v(1);
    end
elseif xq >= x(end) % xq is end point
    if extrapolate
        % Extrapolation function
        vq = v(end) + (v(end)-v(end-1))*(xq-x(end))/(x(end)-x(end-1));
    else
        vq = v(end);
    end
else
    idx1 = f_findIdx1(x,xq);

    if x(idx1) == xq
        vq = v(idx1);
    else
        % Interpolation function
        idx2 = idx1 + 1;
        v1 = v(idx1);
        v2 = v(idx2);
        vq = v1 + (v2-v1)*(xq-x(idx1))/(x(idx2)-x(idx1));
    end
end
end