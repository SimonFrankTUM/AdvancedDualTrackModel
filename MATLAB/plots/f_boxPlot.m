function [] = f_boxPlot(sig,sortSig,numBoxes,xlabelString,ylabelString,titleString)
%%      Box Plot Of Results Errors         %%
% ----------------------------------------- %
% Version: V1.0 - 2024.06.04                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% ----------------------------------------- %

%% Colors
colorsTUM = [   0   101 189; ...
                162 173 0;   ...
                227 114 34;  ...
                100 160 200; ...
                0   82  147; ...
                152 198 234; ...
                0   51  89]./255;

%% Signals
lengthSig   = length(sig);

if lengthSig ~= length(sortSig)
    error('Dimensions of signal and sort signal do not match.')
end

MagSteps= linspace(min(sortSig),max(sortSig),numBoxes+1);

% Group names
names = cell(1,numBoxes);
for i = 1:numBoxes
    names(i) = {[num2str(round(MagSteps(i))) ' - ' num2str(round(MagSteps(i+1)))]};
end

% Sort data into groups
g       = cell(lengthSig,1);
g(:)    = names(1);
for i = 1:numBoxes
    idxRange = sortSig >= MagSteps(i) & sortSig <= MagSteps(i+1);
    g(idxRange) = names(i);
end

limits = [-1 1].*min([5*mean(abs(sig)) 100]);

%% Plot
figure();
try
    boxplot(sig,g,'GroupOrder',names,'DataLim',limits,'ExtremeMode','compress','Colors',colorsTUM(1,:),'Symbol','+k');
catch
    boxplot(sig,g,'DataLim',limits,'ExtremeMode','compress','Colors',colorsTUM(1,:),'Symbol','+k');
    warning('Not all segments appear in the box plot.');
end
ax = gca;
xlabel(xlabelString);
ylabel(ylabelString);
title(titleString);
ax.YGrid = 'on';

lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', colorsTUM(2,:));
set(lines, 'Linewidth', 1.5);

end