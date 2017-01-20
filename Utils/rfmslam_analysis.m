function outdat = rfmslam_analysis(runFolder, outputfolder, initialSigma, edges)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run RFM SLAM on the sim Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build input file name
datafile = strcat(runFolder,'simData.graph');

mkdir(outputfolder);

% construct rfmslam instance with input file name
rfmslam = RFMSLAM2D(initialSigma, datafile, edges);

% solve rfmslam
outdat = rfmslam.solve();

% save this runs output data
save(strcat(outputfolder,'rfmslam_results.mat'),'outdat');

end
