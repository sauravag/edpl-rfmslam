function groundTruth = load_ground_truth_2D(fname)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Insert data into graph by reading
% a text file.
%
% Input:
% baseDir: path to directory containig file
% fname: the name of file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% fprintf('Reading ground truth data data from file...\n')

% open the file
fid = fopen(fname);

groundTruth = struct('path',[],'landmarkIDs',[],'landmarkPos',[],'edges',[]);

% the total number of nodes
numposes = 0;

% the total number of features
numfeats = 0;

% the total number of edges
numedges = 0;

% read it line by line
while true
    
    % read line
    tline = fgetl(fid);
    
    % check if it has anything in it
    if ~ischar(tline);
        break;%end of file
    end
    
    % if it had something, lets see what it had
    % Scan the line upto the first space
    % we will identify what data type this is
    str = textscan(tline,'%s %*[ ]');
    
    if strcmp(str{1},'TRUEVERTEX2')
        
        datastr = textscan(tline,'%s %d %f %f %f');
        numposes = datastr{2};
                
        % store the data in each node
        groundTruth.path = [groundTruth.path, [datastr{3};datastr{4};datastr{5}]];
        
    elseif strcmp(str{1}, 'TRUELANDMARK')
        
        datastr = textscan(tline,'%s %d %f %f');
        
        numfeats = numfeats + 1;
        
        groundTruth.landmarkIDs = [groundTruth.landmarkIDs, datastr{2}];
        groundTruth.landmarkPos = [groundTruth.landmarkPos, [datastr{3};datastr{4}]];

    elseif strcmp(str{1}, 'EDGE')
        
        datastr = textscan(tline,'%s %d %d');
        
        numedges = numedges + 1;
        
        groundTruth.edges(numedges,:) = [datastr{2}, datastr{3}];
    
    elseif strcmp(str{1}, 'TOTALZ')
        
        datastr = textscan(tline,'%s %d');
        
        groundTruth.totalZ = double(datastr{2});
        
    else
        error('Unrecognized data type in file.')
    end
    
end

% close file
fclose(fid);


% fprintf('    Total poses: %d  Total Features: %d \n', numposes,numfeats)
% fprintf('Done reading ground truth data from file.\n')

end