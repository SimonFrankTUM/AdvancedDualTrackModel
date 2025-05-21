%%          Save Results as CSV            %%
% ----------------------------------------- %
% Version: V1.2 - 2024.07.03                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Run this script after a simulation  %
% has finished to save the result as .csv   %
% ----------------------------------------- %

if ~exist('TTin','var') || ~exist('TTout','var')
    error('No simulation results in worksapce.');
end

%% Let user select time range
timeRange = inputdlg({'Start Time','End Time'},...
                        'Optionally trim time range', [1 50; 1 50]);

%% Adjust Time
if ~isempty(timeRange)
    S = timerange(seconds(str2double(timeRange{1})),seconds(str2double(timeRange{2})));

    TTinCut = TTin(S,:);
    TTinCut.Time = TTinCut.Time - seconds(str2double(timeRange{1}));
    TTinCut.Time = duration(TTinCut.Time,'Format','mm:ss.SSS');

    TToutCut = TTout(S,:);
    TToutCut.Time = TToutCut.Time - seconds(str2double(timeRange{1}));
    TToutCut.Time = duration(TToutCut.Time,'Format','mm:ss.SSS');
else
    TTinCut       = TTin;
    TToutCut      = TTout;
    TTinCut.Time  = duration(TTinCut.Time,'Format','mm:ss.SSS');
    TToutCut.Time = duration(TToutCut.Time,'Format','mm:ss.SSS');
end

%% Combine TTs
TTinCut2 = TTinCut;
TTinCut2.Properties.VariableNames = append(TTinCut2.Properties.VariableNames,'_in');
TT = synchronize(TToutCut,TTinCut2,'regular','linear','TimeStep',seconds(1/opts.dataFreq));

%% Save to csv
[savefile,savepath] = uiputfile('*.csv','Save as .csv');
if ~isequal(savefile,0)
    savepathfile = fullfile(savepath,savefile);
    writetimetable(TT,savepathfile,'WriteMode','overwrite');
    disp(['Data saved as ' savefile]);
end
clearvars -except TTin TTout opts