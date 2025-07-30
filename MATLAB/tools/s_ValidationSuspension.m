%%      Suspension Model Validation        %%
% ----------------------------------------- %
% Version: V1.1 - 2024.09.14                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% ----------------------------------------- %
ccc;

%% Settings
% Parameters
veh   = f_vehicleParameters();
tirFA = f_tireParameters(veh.tires{1});
tirRA = f_tireParameters(veh.tires{3});

% Line colors
colorsTUM = [   0   101 189; ...
    162 173 0;   ...
    227 114 34;  ...
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

numSteps= 15;
e_n     = [0; 0; 1]; % Road normal

%% Front axle
if veh.susType_FA == 2
    % Surf view angles
    view1 = 45; view2 = 15;

    % Wheel orientation in design position
    e_yRk = f_zrot(veh.susFL.delta_0)*f_xrot(veh.susFL.gamma_0)*[0;1;0];
    e_xk = cross(e_yRk,e_n); e_xk = e_xk/norm(e_xk); % forward
    e_yk = cross(e_n,e_xk);     % left
    e_zk = cross(e_xk,e_yRk);   % up
    r_WPk= -0.97*tirFA.r_0*e_zk;% Wheel center to contact point

    % Kingpin inclination and caster angle in design position
    r_CFk = veh.susFL.r_VFk - veh.susFL.r_VCk;
    e_CFk = r_CFk/norm(r_CFk); % Kingpin axis
    sigma = atan2d(-e_CFk(2),e_CFk(3)); disp(['Front kingpin inclination sigma = ' num2str(round(sigma,1)) '°']);
    nu    = atan2d(-e_CFk(1),e_CFk(3)); disp(['Front caster angle nu = ' num2str(round(nu,1)) '°']);

    % Caster offset and scrub radius in design position
    r_CPk = veh.susFL.r_VWk + r_WPk - veh.susFL.r_VCk;
    r_SCk = -(e_n.'*r_CPk)/(e_n.'*e_CFk)*e_CFk;
    r_SPk = r_SCk + r_CPk;
    r_VSk = veh.susFL.r_VCk - r_SCk;
    co = -e_xk'*r_SPk; disp(['Front caster offset c = ' num2str(round(co*1E3,1)) 'mm']);
    sr = -e_yk'*r_SPk; disp(['Front scrub radius s = ' num2str(round(sr*1E3,1)) 'mm']);

    % Suspension bodies
    wc_offset = [veh.l_FA; 0; -veh.z_cg];
    wc = veh.susFL.r_VWk -wc_offset; % Wheel Center
    wbl= [veh.susFL.r_VAk veh.susFL.r_VCk veh.susFL.r_VBk] -wc_offset; % Lower wishbone
    wbu= [veh.susFL.r_VDk veh.susFL.r_VFk veh.susFL.r_VEk] -wc_offset; % Upper wishbone
    tr = [veh.susFL.r_VRk veh.susFL.r_VQk] -wc_offset; % Tie rod
    cp = veh.susFL.r_VWk + r_WPk -wc_offset; % Contact point
    sp = r_VSk -wc_offset; % Kingpin road intersection
    kp = [veh.susFL.r_VFk r_VSk] -wc_offset; % Kingpin axis

    % Simulation inputs
    steps = linspace(-1,1,numSteps);
    ctrStp= ceil(numSteps/2);
    phi = veh.susFL.phi_max*steps; u = veh.u_max*steps;

    % Run suspension model
    x_W = zeros(numSteps); y_W=x_W;z_W=x_W;x_P=x_W;y_P=x_W;z_P=x_W;
    delta=x_W; toe=x_W; gamma=x_W;

    for i = 1:numSteps
        for ii = 1:numSteps
            [a_VW, r_VWV, delta(i,ii), dvdphi, domdphi] = f_suspensionDWB(veh.susFL,phi(i),u(ii));
            e_yRV = a_VW*[0;1;0];       % Wheel rotation axis
            r_VPV = r_VWV + a_VW*r_WPk; % Contact point
            x_W(i,ii) = r_VWV(1); y_W(i,ii) = r_VWV(2); z_W(i,ii)=r_VWV(3); % Wheel center coords.
            x_P(i,ii) = r_VPV(1); y_P(i,ii) = r_VPV(2); z_P(i,ii)=r_VPV(3); % Contact point coords.
            toe(i,ii) = atan2(-e_yRV(1), e_yRV(2));  % Toe angle
            gamma(i,ii)= atan2( e_yRV(3), e_yRV(2)); % Inclination angle
        end
    end


    figFront = figure('Name','Front','Units','normalized','OuterPosition',[0 0.5 0.5 0.5]);
    colormap(figFront,'winter');

    % Coordinates
    subplot(131); hold on; axis equal;
    title('Front axle');
    plot3(wbl(1,:),wbl(2,:),wbl(3,:),'o-k','Linewidth',1,'MarkerSize',2);
    plot3(wbu(1,:),wbu(2,:),wbu(3,:),'o-k','Linewidth',1,'MarkerSize',2);
    plot3(tr(1,:),tr(2,:),tr(3,:),'o-k','Linewidth',1,'MarkerSize',2);
    plot3(wc(1,:),wc(2,:),wc(3,:),'o','Linewidth',1,'Color',colors(1,:),'MarkerSize',2);
    plot3(cp(1,:),cp(2,:),cp(3,:),'o','Linewidth',1,'Color',colors(1,:),'MarkerSize',2);
    plot3(sp(1,:),sp(2,:),sp(3,:),'x','Linewidth',1,'Color',colors(2,:),'MarkerSize',2);
    plot3(kp(1,:),kp(2,:),kp(3,:),'--','Linewidth',1,'Color',colors(2,:),'MarkerSize',2);
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(225,10);

    % Gamma
    subplot(132); grid on; hold on;
    surf(u*1e3,(z_W(:,ctrStp)-z_W(ctrStp,ctrStp)).*1e3,rad2deg(gamma)); view(view1,view2);
    title('Inclination angle \gamma');
    xlabel('u in mm');
    ylabel('z_W in mm');
    zlabel('\gamma in °');

    % Delta
    subplot(133); grid on; hold on;
    surf(u*1e3,(z_W(:,ctrStp)-z_W(ctrStp,ctrStp)).*1e3,rad2deg(delta)); view(view1-90,view2);
    title('Steering angle \delta');
    xlabel('u in mm');
    ylabel('z_W in mm');
    zlabel('\delta in °');

    % Ackermann
    dFL = delta(ctrStp,:)-delta(ctrStp,ctrStp);   dFR = -dFL(numSteps:-1:1); % Wheel steering angles
    l   = veh.l_FA+veh.l_RA; t = veh.t_FA; % Wheelbase & track width
    dFRa= atan2(l*tan(dFL), l+t*tan(dFL)); % Ackermann steering
    dFLa= -dFRa(numSteps:-1:1);
    Ackermann = 100*(dFL(end)-dFR(end))./(dFLa(end)-dFRa(end));
    disp(['Ackermann percentage = ' num2str(round(Ackermann)) '%']);

    figAckermann = figure('Name','Ackermann','Units','normalized','OuterPosition',[0 0 0.5 0.5]);
    grid on; hold on;
    plot(u*1e3,rad2deg(dFL),'Color',colors(1,:));
    plot(u*1e3,rad2deg(dFLa),'Color',colors(2,:));
    plot([u(1) u(end)].*1E3,rad2deg(dFLa(ctrStp+1)-dFLa(ctrStp))/u(ctrStp+1)*[u(1) u(end)],'--','Color',colors(3,:));
    title('Front left steering angle');
    xlabel('u in mm');
    ylabel('\delta in °');
    legend('Suspenion','Ackermann','Parallel','Location','northwest');

    % Translations
    figFrontTrans = figure('Name','Translations','Units','normalized','OuterPosition',[0.5 0 0.5 0.5]);
    colormap(figFrontTrans,'winter');

    subplot(131); grid on; hold on;
    surf(u*1e3,phi.*(180/pi),(x_W-x_W(ctrStp,ctrStp)).*1e3); view(view1,view2);
    title('X-Displacement');
    xlabel('u in mm');
    ylabel('\phi in °');
    zlabel('x_W in mm');

    subplot(132); grid on; hold on;
    surf(u*1e3,phi.*(180/pi),(y_W-y_W(ctrStp,ctrStp)).*1e3); view(view1+180,view2);
    title('Y-Displacement');
    xlabel('u in mm');
    ylabel('\phi in °');
    zlabel('y_W in mm');

    subplot(133); grid on; hold on;
    surf(u*1e3,phi.*(180/pi),(z_W-z_W(ctrStp,ctrStp)).*1e3); view(view1+180,view2);
    title('Z-Displacement');
    xlabel('u in mm');
    ylabel('\phi in °');
    zlabel('z_W in mm');

end

%% Rear axle
if veh.susType_RA == 2

    disp('-----------------------------------');

    % Wheel orientation in design position
    e_yRk = f_zrot(veh.susRL.delta_0)*f_xrot(veh.susRL.gamma_0)*[0;1;0];
    e_xk = cross(e_yRk,e_n); e_xk = e_xk/norm(e_xk); % forward
    e_yk = cross(e_n,e_xk);     % left
    e_zk = cross(e_xk,e_yRk);   % up
    r_WPk= -0.97*tirFA.r_0*e_zk;% Wheel center to contact point

    % Kingpin inclination and caster angle in design position
    r_CFk = veh.susRL.r_VFk - veh.susRL.r_VCk;
    e_CFk = r_CFk/norm(r_CFk); % Kingpin axis
    sigma = atan2d(-e_CFk(2),e_CFk(3)); disp(['Rear kingpin inclination sigma = ' num2str(round(sigma,1)) '°']);
    nu    = atan2d(-e_CFk(1),e_CFk(3)); disp(['Rear caster angle nu = ' num2str(round(nu,1)) '°']);

    % Caster offset and scrub radius in design position
    r_CPk = veh.susRL.r_VWk + r_WPk - veh.susRL.r_VCk;
    r_SCk = -(e_n.'*r_CPk)/(e_n.'*e_CFk)*e_CFk;
    r_SPk = r_SCk + r_CPk;
    r_VSk = veh.susRL.r_VCk - r_SCk;
    co = -e_xk'*r_SPk; disp(['Rear caster offset c = ' num2str(round(co*1E3,1)) 'mm'])
    sr = -e_yk'*r_SPk; disp(['Rear scrub radius s = ' num2str(round(sr*1E3,1)) 'mm'])

    % Suspension bodies
    wc_offset = [-veh.l_RA; 0; -veh.z_cg];
    wc = veh.susRL.r_VWk -wc_offset; % Wheel Center
    wbl= [veh.susRL.r_VAk veh.susRL.r_VCk veh.susRL.r_VBk] -wc_offset; % Lower wishbone
    wbu= [veh.susRL.r_VDk veh.susRL.r_VFk veh.susRL.r_VEk] -wc_offset; % Upper wishbone
    tr = [veh.susRL.r_VRk veh.susRL.r_VQk] -wc_offset; % Tie rod
    cp = veh.susRL.r_VWk + r_WPk -wc_offset; % Contact point
    sp = r_VSk -wc_offset; % Kingpin road intersection
    kp = [veh.susRL.r_VFk r_VSk] -wc_offset; % Kingpin axis

    % Simulation inputs
    steps = linspace(-1,1,numSteps);
    ctrStp= ceil(numSteps/2);
    phi = veh.susRL.phi_max*steps;

    % Run suspension model
    x_W = zeros(numSteps,1); y_W=x_W;z_W=x_W;x_P=x_W;y_P=x_W;z_P=x_W;
    delta=x_W; toe=x_W; gamma=x_W;

    for i = 1:numSteps
        [a_VW, r_VWV, delta(i), dvdphi, domdphi] = f_suspensionDWB(veh.susRL,phi(i),0);
        e_yRV = a_VW*[0;1;0];       % Wheel rotation axis
        r_VPV = r_VWV + a_VW*r_WPk; % Contact point
        x_W(i) = r_VWV(1); y_W(i) = r_VWV(2); z_W(i)=r_VWV(3); % Wheel center coords.
        x_P(i) = r_VPV(1); y_P(i) = r_VPV(2); z_P(i)=r_VPV(3); % Contact point coords.
        toe(i) = atan2(-e_yRV(1), e_yRV(2));  % Toe angle
        gamma(i)= atan2( e_yRV(3), e_yRV(2)); % Inclination angle
    end

    figRear = figure('Name','Rear','Units','normalized','OuterPosition',[0.5 0.5 0.5 0.5]);
    colormap(figRear,'winter');

    % Coordinates
    subplot(131); hold on; axis equal;
    title('Rear axle');
    plot3(wbl(1,:),wbl(2,:),wbl(3,:),'o-k','Linewidth',1,'MarkerSize',2);
    plot3(wbu(1,:),wbu(2,:),wbu(3,:),'o-k','Linewidth',1,'MarkerSize',2);
    plot3(tr(1,:),tr(2,:),tr(3,:),'o-k','Linewidth',1,'MarkerSize',2);
    plot3(wc(1,:),wc(2,:),wc(3,:),'o','Linewidth',1,'Color',colors(1,:),'MarkerSize',2);
    plot3(cp(1,:),cp(2,:),cp(3,:),'o','Linewidth',1,'Color',colors(1,:),'MarkerSize',2);
    plot3(sp(1,:),sp(2,:),sp(3,:),'x','Linewidth',1,'Color',colors(2,:),'MarkerSize',2);
    plot3(kp(1,:),kp(2,:),kp(3,:),'--','Linewidth',1,'Color',colors(2,:),'MarkerSize',2);
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(240,10);

    % Gamma
    subplot(132); grid on; hold on;
    plot((z_W-z_W(ctrStp)).*1e3,rad2deg(gamma),'Color',colors(1,:));
    title('Inclination angle \gamma');
    xlabel('z_W in mm');
    ylabel('\gamma in °');

    % Delta
    subplot(133); grid on; hold on;
    plot((z_W-z_W(ctrStp)).*1e3,rad2deg(delta),'Color',colors(1,:));
    title('Steering angle \delta');
    xlabel('z_W in mm');
    ylabel('\delta in °');

end