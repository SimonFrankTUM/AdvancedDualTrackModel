%%              Road creation              %%
% ----------------------------------------- %
% Version: V1.2 - 2025.07.20                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Z is in m while x/y are in 0.1m!    %
% ----------------------------------------- %
ccc;

%% Road height points
% Narrow road
% l = [-2;    100];   % Start and end in m
% w = [-2;    2];     % Side edges in m

% Plane
% l = [-200;  200];   % Start and end in m
% w = [-200;  200];   % Side edges in m


% Distance
% s = l(1):0.1:l(end); % m

% Flat road
% leftRoadEdge    = s.*0; 
% rightRoadEdge   = leftRoadEdge;

% Incline
% leftRoadEdge    = 0.03.*s; 
% rightRoadEdge   = leftRoadEdge; 

% Decline
% leftRoadEdge    = -0.03.*s; 
% rightRoadEdge   = leftRoadEdge; 

% Asym. sine waves
% leftRoadEdge    = 0.05*sin(0.1*pi*s);
% rightRoadEdge   =-0.05*sin(0.1*pi*s);

% Sym. cosine waves
% leftRoadEdge    = 5*cos(1E-2*pi*s);
% rightRoadEdge   = leftRoadEdge;

% Interpolation to 0.1m x-y grid
% h = [rightRoadEdge; leftRoadEdge];
% x = (s(1):0.1:s(end)).*10;
% y = (w(1):0.1:w(end)).*10;
% [X,Y] = meshgrid(x,y);
% Z = interp2(s.*10,w.*10,h,X,Y,'makima')';

%% Random bumpy surface
planeSize   = 65;
maxBumpHeight  = 0.025;
smoothFac = 0.95;

raw_surf = maxBumpHeight.*rand(planeSize*10+1,"double") - maxBumpHeight/2;
x = (-5:0.1:planeSize-5).*10;
y = (-floor(planeSize/2):0.1:ceil(planeSize/2)).*10;
[X,Y] = meshgrid(x,y);
smoothTemp = smoothdata(raw_surf,1,"loess","SmoothingFactor",smoothFac);
Z = smoothdata(smoothTemp,2,"loess","SmoothingFactor",smoothFac);

%% Mu points
% Constant mu
Mu = 1.0.*ones(size(Z));

% Mu split
% x_mu = [l(1) l(end)].*10;
% y_mu = [w(1) 0 1E-3 w(end)].*10;
% mu_right = [0.67 0.67];
% mu_left  = [1.0 1.0];
% mu = [mu_right; mu_right; mu_left; mu_left];
% Mu = interp2(x_mu,y_mu,mu,X,Y,'nearest')';

% Mu gradient
% x_mu = [l(1) l(end)].*10;
% y_mu = [w(1) w(end)].*10;
% mu_right = [0.67 0.67];
% mu_left  = [1.0 1.0];
% mu = [mu_right; mu_left];
% Mu = interp2(x_mu,y_mu,mu,X,Y,'linear')';

% Mu step
% x_mu = [l(1) l(end)/2 l(end)/2+0.1 l(end)].*10;
% y_mu = [w(1) w(end)].*10;
% mu_right = [1.0 1.0 0.67 0.67];
% mu_left  = [1.0 1.0 0.67 0.67];
% mu = [mu_right; mu_left];
% Mu = interp2(x_mu,y_mu,mu,X,Y,'linear')';

%% Plot
figure("Name","Road");

% Height
subplot(211);
surf(x.*0.1,y.*0.1,Z','EdgeColor','none');
axis equal;
xlabel('x in m');
ylabel('y in m');
zlabel('z in m');
title('Road height');

% Mu
subplot(212);
surf = pcolor(x.*0.1,y.*0.1,Mu'.*100);
surf.EdgeColor = 'none';
axis equal;
xlabel('x in m');
ylabel('y in m');
title('\mu_N in %');
colorbar;

%% Save to csv
[savefile,savepath] = uiputfile('*.mat','Save road as .mat');
if isequal(savefile,0)
    return
end
savepathfile = fullfile(savepath,savefile);
save(savepathfile,'x','y','Z','Mu');