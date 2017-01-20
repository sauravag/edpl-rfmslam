%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Vikram Shree, Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function detect_better_method_in_case(baseDirectory)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function runs the comparative analysis
% Give it the directory path containing all the runs,
% it will go into each run folder, then do
% RFM-SLAM and GTSAM and write out summaries
% along with the computed data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


load(strcat(baseDirectory,'rfmslamResults.mat'),'rfmslamResults');

load(strcat(baseDirectory,'gtsamresults.mat'),'gtsamResults');

sumRFMRMS = 0;
sumGTSAMRMS = 0;

for i=1:length(rfmslamResults)
    
    if  ~isempty(rfmslamResults(i).rmsError)
        
        sumRFMRMS = sumRFMRMS + rfmslamResults(i).rmsError(2);
    else
        warning('RFMSLAM did not have any results in run %d in folder: %s \n',i, baseDirectory(end-10:end))
    end
  
end

lessCounter = 0;

for i=1:length(gtsamResults)
    
    if  ~isempty(gtsamResults(i).rmsError)
        sumGTSAMRMS = sumGTSAMRMS + gtsamResults(i).rmsError(2);
    else
        %warning('GTSAM did not have any results in run %d in folder: %s \n',i, baseDirectory(end-10:end))
        lessCounter = lessCounter +1;
    end
end

avgGTSAM = sumGTSAMRMS / (length(gtsamResults)-lessCounter);
avgRFMSLAM = sumRFMRMS / length(rfmslamResults);

fprintf('RFMSLAM Error: %f  GTSAM Error: %f ', avgRFMSLAM, avgGTSAM)

if  avgGTSAM < avgRFMSLAM
    fprintf('===>GTSAM on average did better by: %f %% in Case: %s \n', (avgRFMSLAM - avgGTSAM)*100/avgGTSAM, baseDirectory(end-10:end))
else
    fprintf('===>RFMSLAM on average did better by: %f %% in Case: %s \n', (avgGTSAM - avgRFMSLAM)*100/avgRFMSLAM, baseDirectory(end-10:end))
    
end





end