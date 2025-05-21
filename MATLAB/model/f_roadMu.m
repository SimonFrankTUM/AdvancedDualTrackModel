function mu_R = f_roadMu(road,x_0R,y_0R,t)
%%          Local Road Mu                  %%
% ----------------------------------------- %
% Version: V1.0 - 2024.06.21                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Z is in m while x/y are in 0.1m!    %
% Input:                                    %
%   road    -   struct of road              %
%               x, y, and Z data            %
%   x_0R    -   x coordinate on road        %
%   y_0R    -   y coordinate on road        %
%   t       -   current simulation time     %
% Output:                                   %
%   mu_R    -   local friction coefficient  %
% ----------------------------------------- %

xq = x_0R*10; % Convert to 0.1m
yq = y_0R*10; % Convert to 0.1m

if xq<road.x(1) || xq>road.x(end) || yq<road.y(1) || yq>road.y(end)
    error([num2str(t) 's: Vehicle has left the road.']);
end

% Edge coordinates
xIdx1 = floor(xq - road.x(1)) +1;
yIdx1 = floor(yq - road.y(1)) +1;

xqOnXGrid = xq == road.x(xIdx1);
yqOnYGrid = yq == road.y(yIdx1);

% Interpolation functions
if xqOnXGrid && ~yqOnYGrid
    mu_R = f_LUT1D(road.y,road.Mu(xIdx1,:),yq,false);
elseif ~xqOnXGrid && yqOnYGrid
    mu_R = f_LUT1D(road.x,road.Mu(:,yIdx1),xq,false);
elseif xqOnXGrid && yqOnYGrid
    mu_R = road.Mu(xIdx1,yIdx1);
else
    xIdx2 = xIdx1 + 1;
    yIdx2 = yIdx1 + 1;
    x1 = road.x(xIdx1);
    x2 = road.x(xIdx2);
    y1 = road.y(yIdx1);
    y2 = road.y(yIdx2);

    mu_R = 1/((x2-x1)*(y2-y1)) * [x2-xq xq-x1] * ...
        [road.Mu(xIdx1,yIdx1) road.Mu(xIdx1,yIdx2); road.Mu(xIdx2,yIdx1) road.Mu(xIdx2,yIdx2)] * ...
        [y2-yq; yq-y1];
end
end