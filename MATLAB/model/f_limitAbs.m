function [val, limitActive] = f_limitAbs(val0, limit)
% Limit each absolute value of val0 to limit
val = val0;
limitActive = false;

for i = 1:length(val0)
    if abs(val0(i)) > limit
        val(i) = limit*sign(val0(i));
        limitActive = true;
    end
end

end