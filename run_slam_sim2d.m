%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
close all;
% clear classes
% clear functions;
dbstop if error;
set(0,'DefaultFigureWindowStyle','docked')

CREATE_OUTPUT_DIRECTORY = 1; % set to 1 for writing output to folder

addpath(genpath('./'));

% Add manopt to path if not available
if exist('manopt_version', 'file') ~= 2
    addpath(genpath('../../Softwares/CircStat2012a'))
end

fname = 'MapL1';

inpDat = load(['./Environment/',fname,'.mat']);
nsims = 1;

time = clock; % Gets the current time as a 6 element vector

newFolderName = [fname,'_',...
    num2str(time(1)),'_',... % Returns year as character
    num2str(time(2)),'_',... % Returns month as character
    num2str(time(3)),'_',... % Returns day as char
    num2str(time(4)),'_',... % resturns hour as char..
    num2str(time(5)),'_',...
    num2str(time(6))]; %returns minute as char

% Lets check for platform type i.e., Windows, Linux and define base folder
% accordingly
% base diretory where runs live
if isunix ==1
    [~,username] = system('whoami');
    baseDirectory = ['/home/',username(1:end-1),'/MATLAB/'];
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = ['/Users/',username(1:end-1),'/Documents/MATLAB/'];
    end
end

% Create new directory
if CREATE_OUTPUT_DIRECTORY
    fstat = mkdir(baseDirectory,newFolderName);
    
    % if unsuccessful, exit
    if fstat == 0
        error('Could not create directory to save files');
    end
end

outDatPath = strcat(baseDirectory,newFolderName,'/');

odoNL = 4; % odometery noise level
rbNL = 4; % range bearing noise level

fprintf('Odo Noise Level = %d \n', odoNL);
fprintf('RB Noise Level = %d \n', rbNL);

for i = 1:nsims
    
    if CREATE_OUTPUT_DIRECTORY
        mkdir(outDatPath,['run',num2str(i)]);
    end
    
   slam_sim2d(i, inpDat, odoNL, rbNL, [outDatPath,'run',num2str(i),'/']);
    
    close all;
end

fid = fopen(strcat(outDatPath,'noiseLevels.txt'),'wt');
fprintf(fid, '=========NOISE LEVELS=======\n');
fprintf(fid, 'Odo Noise Level = %d \n', odoNL);
fprintf(fid, 'RB Noise Level = %d \n', rbNL);
fclose(fid);

fprintf('Wrote out files to folder: %s \n',outDatPath)
comparative_analysis(outDatPath)