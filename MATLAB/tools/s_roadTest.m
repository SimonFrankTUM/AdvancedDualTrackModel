%%              Road Model Test            %%
% ----------------------------------------- %
% Version: V1.1 - 2024.09.14                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Test road height look up and        %
% compare to MATLAB internal function       %
% ----------------------------------------- %
ccc;

%% Colors
colorsTUM = [   0   101 189; ...
    162 173 0;   ...
    227 114 34;  ...Fv
    100 160 200; ...
    0   82  147; ...
    152 198 234; ...
    0   51  89]./255;
% colorsMatlab = [0       0.447   0.741
%                 0.850   0.325   0.098
%                 0.929   0.694   0.125
%                 0.494   0.184   0.556
%                 0.466   0.674   0.188
%                 0.301   0.745   0.933
%                 0.635   0.078   0.184];
% fiftyShadesOfBlue = [1 0.67 0.33 0.75 0.5 0.25]'*[0 0.6 0.9];
% fiftyShadesOfGrey = [0 0.6 0.2 0.4 0.8]'*ones(1,3);

colors = colorsTUM;

%% Load road
[loadfile,loadpath] = uigetfile('*.mat','Open road file (.mat). Press Cancel to skip.');
if isequal(loadfile,0)
    error('No road file selected');
else
    loadpathfile = fullfile(loadpath,loadfile);
    road = load(loadpathfile);
    opts.use3DRoad = true;
    disp(['Road file: ' loadfile]);
end

%% Query point
xq = 91.04324; % m
yq = -1.07355; % m

tic
zTest = f_roadHeight(road,xq,yq,0);
fprintf('Custom function: ');
toc

tic
zValidation = interp2(road.x.*0.1,road.y.*0.1,road.Z',xq,yq,'linear');
fprintf('Matlab function: ');
toc

zError = zTest - zValidation;

mu_R = f_roadMu(road,xq,yq,0);

disp('z Test:'); disp(zTest);
disp('z Validation:'); disp(zValidation);
disp('z Error:'); disp(zError);
disp('Floating-point relative accuracy:'); disp(eps);
disp('Local mu:'); disp(mu_R);

%% Tire road contact
delta = deg2rad(-30);
tir.r_0 = 0.3469;
tir.b_0 = 0.215;
tir.R_N = 0;
r_0M0 = [xq; yq; zTest+0.195];
v_0M0 = [0; 0; 0];
omega_0U0 = [0; 0; 0];
e_yR0 = f_zrot(delta)*[0; 1; 0];
[v_0QQ, A_0Q, gamma, r_st, dz, w, r_MQ0] = f_roadContact(road, tir, r_0M0, v_0M0, omega_0U0, e_yR0, opts, 0);

%% Plot
close all;
figure('Name','Road');

% Road surface
surf(road.x*0.1,road.y*0.1,road.Z','EdgeAlpha',0.25,'FaceColor',[0.8 .8 .8]);
axis equal; hold on;

% Test points
plot3(xq,yq,zTest,'rx');
plot3(xq,yq,zValidation,'go');

% Vectors
quiver3(xq,yq,zTest,A_0Q(1,1),A_0Q(2,1),A_0Q(3,1),0.33,'LineWidth',1.5,'MaxHeadSize',1,'Color',colors(1,:)); % x
quiver3(xq,yq,zTest,A_0Q(1,2),A_0Q(2,2),A_0Q(3,2),0.33,'LineWidth',1.5,'MaxHeadSize',1,'Color',colors(7,:)); % y
quiver3(xq,yq,zTest,A_0Q(1,3),A_0Q(2,3),A_0Q(3,3),0.33,'LineWidth',1.5,'MaxHeadSize',1,'Color',colors(3,:)); % z

% Plot settings
title('Road Surface');
xlabel('x in m');
ylabel('y in m');
zlabel('z in m');
axis equal;
range = 1;
xlim([xq-range/2 xq+range/2]);
ylim([yq-range/2 yq+range/2]);
zlim([zTest-range/2 zTest+range/2]);
