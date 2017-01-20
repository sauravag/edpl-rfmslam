%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read graph from file and perform GraphSLAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function outdat = gtsam_analysis_dog(baseDirectory,storeFolderName, folderName, nRun, absErrorTol, relErrorTol, maxIterat)
% %% Dogleg Optimizer Parameters

outdat = struct();
import gtsam.*

filePath  = strcat(baseDirectory,folderName);

%% Find data file
datafile = strcat(filePath,'run',num2str(nRun),'/plumDat.graph');

%% Initialize graph, initial estimate, and odometry noise
% model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 2*pi/180]);
% [graph,initial] = load2D(datafile, model);
[graph,initial] = load2D(fullfile(datafile));

fName = strcat(filePath,'run',num2str(nRun),'/plumDat_heading.txt');
fileID = fopen(fName,'r');
if fileID ~= -1
    %% Taking the Yaw Angle estimate from the data file
    sigmaHeading = fscanf(fileID,'%f',1);
    headingAngle = fscanf(fileID,'%f');
    vertexCount = size(headingAngle);
    fclose(fileID);
    
    %% Providing the prior for the poses in order to incorporate the heading information
    headingNoiseModel  = noiseModel.Diagonal.Sigmas([Inf; Inf; sigmaHeading]);
    % we are not sure that whether GTSAM can handle Inf, so for now we are not using it
    % headingNoiseModel  = noiseModel.Diagonal.Sigmas([1e10; 1e10; sigmaHeading]);
    for i=2:vertexCount
        mean = initial.at(i);
        tempMean = Pose2(mean.x,mean.y,headingAngle(i));
        graph.add(PriorFactorPose2(i, tempMean, headingNoiseModel));
    end
end

%% Add a Gaussian prior on a pose in the middle
priorMean = initial.at(1);
priorNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.5*pi/180]);
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% Optimize using Dogleg optimization with an ordering from colamd
params = DoglegParams;
params.setAbsoluteErrorTol(absErrorTol);
params.setRelativeErrorTol(relErrorTol);
params.setMaxIterations(maxIterat);
% params.setVerbosity('ERROR');
% params.setVerbosityDL('VERBOSE');
params.setOrdering(graph.orderingCOLAMD());
optimizer = DoglegOptimizer(graph, initial, params);

tic
result = optimizer.optimizeSafely;
toc

marginals = Marginals(graph, result);
keys = KeyVector(result.keys);
estimatedPose = utilities.extractPose2(result);
estimatedFeats = utilities.extractPoint2(result);
poseCovariance = [];
featCovariance = [];

for i = 0:keys.size-1
    key = keys.at(i);
    x = result.at(key);
    if isa(x, 'gtsam.Pose2')
        poseCovariance(:,:,end+1) = marginals.marginalCovariance(key);
    end
    if isa(x, 'gtsam.Point2')
        featCovariance(:,:,end+1) = marginals.marginalCovariance(key);
    end
end
featCovariance(:,:,1) = [];
poseCovariance(:,:,1) = [];
outdat.estimatedPose = estimatedPose;
outdat.estimatedFeats = estimatedFeats;
outdat.poseCovariance = poseCovariance;
outdat.featCovariance = featCovariance;
save(strcat(baseDirectory,storeFolderName,'run',num2str(nRun),'/GTSAM_results.mat'),'estimatedPose','estimatedFeats','poseCovariance','featCovariance');

%% Plot Initial Estimate
mfh = figure;
plot2DTrajectory(initial, 'b-'); axis equal
%% Plot Covariance Ellipses
figure(mfh);
hold on
plot2DTrajectory(result, 'm');%, marginals);
plot2DPoints(result, 'm', marginals);
figName = strcat(baseDirectory,storeFolderName,'run',num2str(nRun),'/GTSAM_solution.fig');
axis tight
axis equal
hold off;
saveas(mfh,figName);
end
