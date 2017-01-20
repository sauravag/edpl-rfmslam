function success = writegroundtruth2file(baseDir, robotTraj, featuresSeen, allLandmarks, gEdges,varargin)

nVarargs = length(varargin);

% create file name
fName = strcat(baseDir,'groundTruth.graph');

% open file
fileID = fopen(fName,'wt');

% featuresSeen contains all the features seen in the order they appear
uniqueFeaturesSeen = sort(featuresSeen);

% write traj
for i = 1:size(robotTraj,2)
    fprintf(fileID,'TRUEVERTEX2 %d %f %f %f\n',i,robotTraj(:,i)');
end

% write out unique landmarks
for j = 1:size(uniqueFeaturesSeen,2)
    fprintf(fileID,'TRUELANDMARK %d %f %f\n',uniqueFeaturesSeen(j),allLandmarks(:,uniqueFeaturesSeen(j))');
end

% write edges
for e = 1:size(gEdges,1)    
    fprintf(fileID,'EDGE %d %d\n', gEdges(e,1), gEdges(e,2));   
end

if nVarargs > 0
    fprintf(fileID,'TOTALZ %d\n', varargin{1});
end

fclose(fileID)

if fileID == -1
    success = 0;
    return;
else
    success = 1;
end

end