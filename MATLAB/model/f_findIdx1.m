function idx1 = f_findIdx1(x,xq)
% Find last index idx1 where xq >= x
idx1 = 1;

for i = 1:(length(x)-1)
    if xq < x(i)
        return
    end
    idx1 = i;
end

end