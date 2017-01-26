%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
close all;
clear functions;
clear classes;
clear variables;

addpath(genpath('./'));

% base diretory where runs live
baseDirectory = '/Users/sauravagarwal/Box Sync/RFM-SLAM/MatlabData/ICRA2017/';
    
foldernamesL1 = {'MapL1NL22/','MapL1NL24/','MapL1NL26/','MapL1NL28/',...
                 'MapL1NL42/','MapL1NL44/','MapL1NL46/','MapL1NL48/',...
                 'MapL1NL62/','MapL1NL64/','MapL1NL66/','MapL1NL68/',...
                 'MapL1NL82/','MapL1NL84/','MapL1NL86/','MapL1NL88/'};

foldernamesS4 = {'MapS4NL22/','MapS4NL24/','MapS4NL26/','MapS4NL28/',...
                 'MapS4NL42/','MapS4NL44/','MapS4NL46/','MapS4NL48/',...
                 'MapS4NL62/','MapS4NL64/','MapS4NL66/','MapS4NL68/',...
                 'MapS4NL82/','MapS4NL84/','MapS4NL86/','MapS4NL88/'};
            
foldernames = {foldernamesL1{:},foldernamesS4{:}};

for f = 1:length(foldernames)
%     comparative_analysis(strcat(baseDirectory,foldernames{f}));
    flag_weird_cases(strcat(baseDirectory,foldernames{f}));
end