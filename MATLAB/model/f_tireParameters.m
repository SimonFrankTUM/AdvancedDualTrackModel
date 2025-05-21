function tir = f_tireParameters(tire)
%%              Tire Parameters            %%
% ----------------------------------------- %
% Version: V2.0 - 2024.09.12                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   tire    -   name of chosen tire         %
%               parameter set as char       %
% Output:                                   %
%   tir     -   struct containing tire      %
%               parameters for one tire     %
% ----------------------------------------- %
tir = struct();

try
    run(['.' filesep 'tires' filesep tire '.m']);
catch
    error('Unknown tire file selected.');
end

if 2*tir.F_xMA1 < tir.F_xMA2 || 2*tir.F_xMB1 < tir.F_xMB2 || 2*tir.F_yM1 < tir.F_yM2
    error('Implausible tire data: Condition 2*F_M1>F_M2 not met.');
end

if tir.dF_x01 < 2*tir.F_xMA1/tir.s_xMA1 || tir.dF_x01 < 2*tir.F_xMB1/tir.s_xMB1 || ...
        tir.dF_x02 < 2*tir.F_xMA2/tir.s_xMA2 || tir.dF_x02 < 2*tir.F_xMB2/tir.s_xMB2 || ...
        tir.dF_y01 < 2*tir.F_yM1/tir.s_yM1 || tir.dF_y02 < 2*tir.F_yM2/tir.s_yM2
    error('Implausible tire data: Condition dF_0>2*F_M/s_M not met.');
end

end