function idx1 = f_findIdx1(x,xq)
% Find last index idx1 where xq >= x

for i = 1:(length(x)-1)
    if xq < x(i)
        return
    end
    idx1 = i;
end

end