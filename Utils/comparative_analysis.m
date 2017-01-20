%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Vikram Shree, Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function comparative_analysis(baseDirectory)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function runs the comparative analysis
% Give it the directory path containing all the runs,
% it will go into each run folder, then do
% RFM-SLAM and GTSAM and write out summaries
% along with the computed data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Start RFMSLAM vs. GTSAM comparison...  \n');

%% Optimizer Parameters
maxIterat = 100;
absErrorTol = 1e-12;
relErrorTol = 1e-12;
initialSigma = [0.01;0.01;0.01*pi/180]; % Initial uncertainty at pose in ICRA SIMS [0.01;0.01;0.05*pi/180]
doRFMSLAM = 1;
doGTSAM = 0;

%% Setup the paths

% Get list of files in base folder
fileList = dir(baseDirectory);

% Get a logical vector that tells which is a directory.
dirFlags = [fileList.isdir];

% Extract only those that are directories.
subFolders = fileList(dirFlags);

nRuns = 0;

fprintf('Getting list of run folders...  \n');
% get number of folder with the string run in them.
for k = 1 : length(subFolders)
    str = textscan(subFolders(k).name,'%s %*[.]');
    
    idx = regexp(str{1},'run');
    if ~isempty(idx{1})
        nRuns = nRuns + 1;
    end
end

fprintf('Number of runs to analyze: %d \n', nRuns)

%% RFM-SLAM
if doRFMSLAM
    rfmslamResults = struct(); % the RFMSLAM Results
    sumRMSRFMSLAM = 0;
    sumRMSlmRFMSLAM_x = 0;
    sumRMSlmRFMSLAM_y = 0;
    sumOptimizationTimeRFMSLAM = 0;
    storeFolderName = 'rfmslam/';
    fileID = fopen(strcat(baseDirectory,'rfmslamsummary.txt'),'wt');
    
    % Read in ground truth data
    groundTruth = load_ground_truth_2D(strcat(baseDirectory,'groundTruth.graph'));
    
    for i=1:nRuns
        
        fprintf('\n \n \n --- RFMSLAM ANALYSIS OF RUN #%d IN: %s --- \n \n \n',i,baseDirectory)
        
        % the path of this run folder
        runFolder = strcat(baseDirectory,'run',num2str(i),'/');        
        
        % path of gtsam output folder inside run folder
        outputfolder = strcat(runFolder,storeFolderName);
        
        outdat = rfmslam_analysis(runFolder, outputfolder, initialSigma, groundTruth.edges);
%         load(strcat(outputfolder,'rfmslam_results.mat'),'outdat');
        
        rfmslamResults(i).estimatedPose  = outdat.estimatedPose;
        rfmslamResults(i).estimatedFeats = outdat.estimatedFeats;
        rfmslamResults(i).poseCovariance = outdat.poseCovariance;
        rfmslamResults(i).featCovariance = outdat.featCovariance;
        rfmslamResults(i).tOrientationOpt  = outdat.tOrientationOpt;
        rfmslamResults(i).totalTime  = outdat.totalTime;
        
        % Compute localization Error in GTSAM for this run
        dx = groundTruth.path(1,:) - rfmslamResults(i).estimatedPose(1,:);
        dy = groundTruth.path(2,:) - rfmslamResults(i).estimatedPose(2,:);
        dTheta = pi_to_pi(groundTruth.path(3,:) - rfmslamResults(i).estimatedPose(3,:))*180/pi;
        
        rfmslamResults(i).localizationError = [dx; dy; dTheta];
        
        % Compute RMS Error in GTSAM for this run
        rfmslamResults(i).rmsError = [sqrt(mean(dTheta.^2)) sqrt(mean(dx.^2 + dy.^2))];
        
        sumRMSRFMSLAM = rfmslamResults(i).rmsError(2) + sumRMSRFMSLAM;
        
        % Compute the error in landmark localization
        % The IDs of the landmarks in GTSAM are in the order of the global IDs.
        lmX = rfmslamResults(i).estimatedFeats(1,:) - groundTruth.landmarkPos(1,:);
        lmY = rfmslamResults(i).estimatedFeats(2,:) - groundTruth.landmarkPos(2,:);
        rfmslamResults(i).rmsLmErrorX = sqrt(mean(lmX.^2));
        rfmslamResults(i).rmsLmErrorY = sqrt(mean(lmY.^2));
        
        sumRMSlmRFMSLAM_x = sumRMSlmRFMSLAM_x + rfmslamResults(i).rmsLmErrorX;
        sumRMSlmRFMSLAM_y = sumRMSlmRFMSLAM_y + rfmslamResults(i).rmsLmErrorY;
        sumOptimizationTimeRFMSLAM = sumOptimizationTimeRFMSLAM + outdat.totalTime;
        
        % Write data to file
        fprintf(fileID, '================ RUN %d =============== \n', i);
        fprintf(fileID, 'RMS Heading Error = %f degrees \n', rfmslamResults(i).rmsError(1));
        fprintf(fileID, 'RMS Localization Error = %f \n', rfmslamResults(i).rmsError(2));
        fprintf(fileID, 'Terminal Localization Error = %f \n', rfmslamResults(i).localizationError(end,2));
        fprintf(fileID, 'Feature Localization Error X = %f \n', rfmslamResults(i).rmsLmErrorX);
        fprintf(fileID, 'Feature Localization Error Y = %f \n', rfmslamResults(i).rmsLmErrorY);
        fprintf(fileID, 'Time to Solve Orientation: %f seconds \n', rfmslamResults(i).tOrientationOpt);
        fprintf(fileID, 'Total Time to Solve: %f seconds \n', rfmslamResults(i).totalTime);
        fprintf(fileID, '---------------------------------------- \n');
                
        % It is faster to not plot now,
        % we can manually plot interesting cases later.
        %     fplum = openfig(strcat(runFolder,'animation',num2str(i),'.fig'));
        %     fgtsam = openfig(strcat(runFolder,'/GTSAM_solution.fig'));
        %
        %     tempLines = findobj(fgtsam,'type','line');
        %     copyobj(tempLines,findobj(fplum,'type','axes'));
        %
        %     saveas(fplum,strcat(baseDirectory,storeFolderName,'run',num2str(i),'/mergedFigure.fig'));
    end
    
    save(strcat(baseDirectory,'rfmslamResults.mat'),'rfmslamResults');
    
    fprintf(fileID,'========== SUMMARY ======== \n');
    fprintf(fileID,'Avg RMS Localization Error = %f \n', sumRMSRFMSLAM / nRuns);
    fprintf(fileID,'Avg RMS Landmark Localization Error in X = %f \n', sumRMSlmRFMSLAM_x / nRuns);
    fprintf(fileID,'Avg RMS Landmark Localization Error in Y = %f \n', sumRMSlmRFMSLAM_y / nRuns);
    fprintf(fileID,'Avg Time for Orientation Optimization and LLSQ = %f \n', sumOptimizationTimeRFMSLAM / nRuns);
    fprintf(fileID,'----------END------------- \n');
    
    fclose(fileID);
end

%% GTSAM
if doGTSAM
    % LevenbergMarquardtOptimizer
    gtsamResults = struct(); % the GTSAM Results
    sumRMSGTSAM = 0;
    sumRMSlmGTSAM_x = 0;
    sumRMSlmGTSAM_y = 0;
    sumOptimizationTimeGTSAM = 0;
    storeFolderName = 'gtsam_lev/';
    fileID = fopen(strcat(baseDirectory,'gtsamsummary.txt'),'wt');
    
    % Read in ground truth data
    groundTruth = load_ground_truth_2D(strcat(baseDirectory,'groundTruth.graph'));
    numUnsolved = 0;
    
    for i=1:1%nRuns
        
        fprintf('\n \n \n ---- GTSAM ANALYSIS OF RUN #%d IN: %s ---- \n \n \n',i, baseDirectory)

        % the path of this run folder
        runFolder = strcat(baseDirectory,'run',num2str(i),'/');               
        
        % path of gtsam output folder inside run folder
        outputfolder = strcat(runFolder,storeFolderName);
        
        try
            outdat = gtsam_analysis_lev(runFolder, outputfolder, initialSigma, absErrorTol,relErrorTol, maxIterat);
            
%             load(strcat(outputfolder,'gtsam_lev_results.mat'),'outdat');
        catch ME
            fprintf('GTSAM could not solve or load this run...\n')
            numUnsolved = numUnsolved + 1;
            continue;
        end
            
        gtsamResults(i).estimatedPose  = outdat.estimatedPose;
        gtsamResults(i).estimatedFeats = outdat.estimatedFeats;
        gtsamResults(i).poseCovariance = outdat.poseCovariance;
        gtsamResults(i).globalposeXYCovariance = outdat.globalposeXYCovariance;
        gtsamResults(i).featCovariance = outdat.featCovariance;
        gtsamResults(i).tOptimization  = outdat.tOptimization;
        
        % Compute localization Error in GTSAM for this run
        dx = groundTruth.path(1,:) - gtsamResults(i).estimatedPose(1,:);
        dy = groundTruth.path(2,:) - gtsamResults(i).estimatedPose(2,:);
        dTheta = pi_to_pi(groundTruth.path(3,:) - gtsamResults(i).estimatedPose(3,:))*180/pi;
        
        gtsamResults(i).localizationError = [dx; dy; dTheta];
        
        % Compute RMS Error in GTSAM for this run
        gtsamResults(i).rmsError = [sqrt(mean(dTheta.^2)) sqrt(mean(dx.^2 + dy.^2))];
        
        sumRMSGTSAM = gtsamResults(i).rmsError(2) + sumRMSGTSAM;
        
        % Compute the error in landmark localization
        % The IDs of the landmarks in GTSAM are in the order of the global IDs.
        lmX = gtsamResults(i).estimatedFeats(1,:) - groundTruth.landmarkPos(1,:);
        lmY = gtsamResults(i).estimatedFeats(2,:) - groundTruth.landmarkPos(2,:);
        gtsamResults(i).rmsLmErrorX = sqrt(mean(lmX.^2));
        gtsamResults(i).rmsLmErrorY = sqrt(mean(lmY.^2));
        
        sumRMSlmGTSAM_x = sumRMSlmGTSAM_x + gtsamResults(i).rmsLmErrorX;
        sumRMSlmGTSAM_y = sumRMSlmGTSAM_y + gtsamResults(i).rmsLmErrorY;
        sumOptimizationTimeGTSAM = sumOptimizationTimeGTSAM + outdat.tOptimization;
        
        % Write data to file
        fprintf(fileID, '================ RUN %d =============== \n', i);
        fprintf(fileID, 'RMS Heading Error = %f \n', gtsamResults(i).rmsError(1));
        fprintf(fileID, 'RMS Localization Error = %f \n', gtsamResults(i).rmsError(2));
        fprintf(fileID, 'Terminal Localization Error = %f \n', gtsamResults(i).localizationError(end,2));
        fprintf(fileID, 'Feature Localization Error X = %f \n', gtsamResults(i).rmsLmErrorX);
        fprintf(fileID, 'Feature Localization Error Y = %f \n', gtsamResults(i).rmsLmErrorY);
        fprintf(fileID, 'Optimization Time: %f seconds \n', gtsamResults(i).tOptimization);
        fprintf(fileID, '---------------------------------------- \n');
        
        % It is faster to not plot now,
        % we can manually plot interesting cases later.
        %     fplum = openfig(strcat(runFolder,'animation',num2str(i),'.fig'));
        %     fgtsam = openfig(strcat(runFolder,'/GTSAM_solution.fig'));
        %
        %     tempLines = findobj(fgtsam,'type','line');
        %     copyobj(tempLines,findobj(fplum,'type','axes'));
        %
        %     saveas(fplum,strcat(baseDirectory,storeFolderName,'run',num2str(i),'/mergedFigure.fig'));
    end
    
    save(strcat(baseDirectory,'gtsamresults.mat'),'gtsamResults');

    fprintf(fileID,'========== SUMMARY ======== \n');
    fprintf(fileID,'Avg RMS Localization Error = %f \n', sumRMSGTSAM / (nRuns-numUnsolved));
    fprintf(fileID,'Avg RMS Landmark Localization Error in X = %f \n', sumRMSlmGTSAM_x / (nRuns-numUnsolved));
    fprintf(fileID,'Avg RMS Landmark Localization Error in Y = %f \n', sumRMSlmGTSAM_y / (nRuns-numUnsolved));
    fprintf(fileID,'Absolute Error Tolerance == %e \n',absErrorTol);
    fprintf(fileID,'Relative Error Tolerance == %e \n',relErrorTol);
    fprintf(fileID,'Max. number of Iterations = %d \n',maxIterat);
    fprintf(fileID,'Avg Time for Optimization = %f \n', sumOptimizationTimeGTSAM / (nRuns-numUnsolved));
    fprintf(fileID,'----------END------------- \n');
    
    fclose(fileID);
end

%% DoglegOptimizer

% if doDogLeg
%     gtsamResults = struct(); % the GTSAM Results
%     sumRMSGTSAM = 0;
%     sumRMSlmGTSAM_x = 0;
%     sumRMSlmGTSAM_y = 0;
%     storeFolderName = strcat(folderName,'dog','/','maxIterat',num2str(maxIterat),'/','absErrorTol',num2str(absErrorTol),'/','relErrorTol',num2str(relErrorTol),'/');
%     mkdir(strcat(baseDirectory,storeFolderName));
%     fileID = fopen(strcat(baseDirectory,storeFolderName,'gtsamsummary.txt'),'wt');
%     i = 1;
%
%     for i=1:nRuns
%         mkdir(strcat(baseDirectory,storeFolderName,'run',num2str(i)));
%         outdat = gtsam_analysis_dog(baseDirectory,storeFolderName, folderName,i,absErrorTol,relErrorTol, maxIterat);
%         gtsamResults(i).estimatedPose =  outdat.estimatedPose;
%         gtsamResults(i).estimatedFeats = outdat.estimatedFeats;
%         gtsamResults(i).poseCovariance = outdat.poseCovariance;
%         gtsamResults(i).featCovariance = outdat.featCovariance;
%
%         % Compute localization Error in GTSAM for this run
%         dx = plumResults(i).truePath(:,2) - gtsamResults(i).estimatedPose(:,1);
%         dy = plumResults(i).truePath(:,3) - gtsamResults(i).estimatedPose(:,2);
%         dTheta = plumResults(i).truePath(:,4) - gtsamResults(i).estimatedPose(:,3);
%
%         gtsamResults(i).localizationError = [dTheta sqrt(dx.^2 + dy.^2)];
%
%         % Compute RMS Error in GTSAM for this run
%         gtsamResults(i).rmsError = [sqrt(mean(dTheta.^2)) sqrt(mean(gtsamResults(i).localizationError(:,2).^2))];
%
%         sumRMSGTSAM = gtsamResults(i).rmsError(2) + sumRMSGTSAM;
%
%         % Compute the error in landmark localization
%         lmX = gtsamResults(i).estimatedFeats(:,1) - map.lm(1,gtsamLMID(i).landmarkID(:))';
%         lmY = gtsamResults(i).estimatedFeats(:,2) - map.lm(2,gtsamLMID(i).landmarkID(:))';
%         gtsamResults(i).rmsLmErrorX = sqrt(mean(lmX.^2));
%         gtsamResults(i).rmsLmErrorY = sqrt(mean(lmY.^2));
%
%         sumRMSlmGTSAM_x = sumRMSlmGTSAM_x + gtsamResults(i).rmsLmErrorX;
%         sumRMSlmGTSAM_y = sumRMSlmGTSAM_y + gtsamResults(i).rmsLmErrorY;
%
%         % Write data to file
%         fprintf(fileID, '===== RUN %u ===== \n', i);
%         fprintf(fileID, 'RMS Localization Error = %f \n', gtsamResults(i).rmsError(2));
%         fprintf(fileID, 'Terminal Localization Error = %f \n', gtsamResults(i).localizationError(end,2));
%         fprintf(fileID, '------------------ \n');
%
%         fplum = openfig(strcat(baseDirectory,folderName,'run',num2str(i),'/','animation',num2str(i),'.fig'));
%         fgtsam = openfig(strcat(baseDirectory,storeFolderName,'run',num2str(i),'/GTSAM_solution.fig'));
%
%         tempLines = findobj(fgtsam,'type','line');
%         copyobj(tempLines,findobj(fplum,'type','axes'));
%
%         saveas(fplum,strcat(baseDirectory,storeFolderName,'run',num2str(i),'/mergedFigure.fig'));
%     end
%
%     save(strcat(baseDirectory,storeFolderName,'gtsamresults.mat'),'gtsamResults');
%
%     fprintf(fileID,'===== SUMMARY ===== \n');
%     fprintf(fileID,'Avg RMS Localization Error = %f \n', sumRMSGTSAM / nRuns);
%     fprintf(fileID,'Avg RMS Landmark Localization Error in X = %f \n', sumRMSlmGTSAM_x / nRuns);
%     fprintf(fileID,'Avg RMS Landmark Localization Error in Y = %f \n', sumRMSlmGTSAM_y / nRuns);
%     fprintf(fileID,'Absolute Error Tolerance == %e \n',absErrorTol);
%     fprintf(fileID,'Relative Error Tolerance == %e \n',relErrorTol);
%     fprintf(fileID,'Max. number of Iterations = %d \n',maxIterat);
%     fprintf(fileID,'----------END------------- \n');
%
%     fclose(fileID);
%
%     fprintf('done \n');
% end

end