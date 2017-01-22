%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Vikram Shree, Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function flag_weird_cases(baseDirectory)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function runs the comparative analysis
% Give it the directory path containing all the runs,
% it will go into each run folder, then do
% RFM-SLAM and GTSAM and write out summaries
% along with the computed data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set which to check
doRFMSLAM = 1;
doGTSAM = 1;

flagThreshPos = 0.02*1000.87;

%% RFM-SLAM
if doRFMSLAM
    
    rff = 0;
    
    load(strcat(baseDirectory,'rfmslamResults.mat'),'rfmslamResults');
    
    if length(rfmslamResults) < 50
        warning('RFMSLAM has results for less than 50 runs in Folder: %s \n', baseDirectory(end-10:end))
    end
    
    fprintf('\n \n ============= Folder %s ========== \n \n', baseDirectory(end-10:end)); 
    
    for i=1:length(rfmslamResults)
        
        % rmsError(1) is theta and rmsError(2) is position
        if  rfmslamResults(i).rmsError(2) > flagThreshPos % || rfmslamResults(i).rmsError(1) > 2.0
            rff = rff + 1;
            fprintf('    RFMSLAM Error: %f Flag Run: %d in folder %s \n', rfmslamResults(i).rmsError(2), i, baseDirectory(end-10:end))
        end
    end
    
    if rff >0
        fprintf('=====> RFMSLAM Number of Failures: %d in folder %s <====== \n \n',rff, baseDirectory(end-10:end)); 
    else
        fprintf('=====> No Weird RFMSLAM Case in folder %s <====== \n', baseDirectory(end-10:end)); 
    end
   
end

%% GTSAM
if doGTSAM

    gtf = 0;
    gtunsolved = 0;
    
    load(strcat(baseDirectory,'gtsamresults.mat'),'gtsamResults');    
    
    if length(gtsamResults) < 50
        warning('GTSAM has results for less than 50 runs in Folder: %s \n', baseDirectory(end-10:end))
    end
    
    for i=1:length(gtsamResults)
        
        if isempty(gtsamResults(i).rmsError)
            gtunsolved = gtunsolved +1;
            %warning('GTSAM does not have results for run number: %d in folder %s',i, baseDirectory)
            continue;
        end
        
        if gtsamResults(i).rmsError(2) > flagThreshPos % || gtsamResults(i).rmsError(1) > 2.0
            gtf = gtf + 1;
            fprintf('    GTSAM Error: %f Flag Run: %d in folder %s \n',gtsamResults(i).rmsError(2), i, baseDirectory(end-10:end))
        end
  
    end
    
    if gtf >0
        fprintf('===> GTSAM Number of Failures: %d Unsolved: %d  in folder %s \n', gtf, gtunsolved, baseDirectory(end-10:end));
    else
        fprintf('=====> No Weird GTSAM Case in folder %s <====== \n', baseDirectory(end-10:end));
    end

end


end