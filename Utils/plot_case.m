%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot a case
% clc;
close all;
clear variables;
set(0,'DefaultFigureWindowStyle','docked')

%% STEP 1
% CHANGE this variable to choose which sim to analyze
datafoldername = 'Map25kF4_2017_1_26_14_33_17.382/';

% CHOOSE which run to plot
runToPlot = 1;

doLFGO = 0;
doRFMSLAM = 0;
doGTSAM = 1;
doTrueMap = 0;

%% STEP 2
% base diretory where runs live
if isunix ==1
    [~,username] = system('whoami');
    baseDirectory = ['/home/',username(1:end-1),'/MATLAB/'];
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = '/Users/sauravagarwal/Box Sync/RFM-SLAM/MatlabData/ICRA2017/';
    end
else % is windows
    baseDirectory = '<###>';
end

% make name of run folder
baseDirectory = strcat(baseDirectory,datafoldername); % concatenate
runFolder = strcat(baseDirectory,'run',num2str(runToPlot),'/');

% the gtsam result file for this run
gtsamfile = strcat(runFolder,'gtsam_lev/gtsam_lev_results.mat');

% the rfmslam run file
rfmslamfile = strcat(runFolder,'rfmslam/rfmslam_results.mat');

% the lfgo run file
lfgofile = strcat(runFolder,'lfgo/lfgo_results.mat');

% the ground truth
groundtruthfile = strcat(baseDirectory,'groundTruth.graph');
groundTruth = load_ground_truth_2D(groundtruthfile);

% the odometery / initial guess
initialGuessTraj = load_initial_guess_from_graph(strcat(runFolder,'simData.graph'));

%% Step 3
% Plotting Params
N = 50;% number of points in ellipse drawing
confidence = 0.9973; % draw the (2-sigma) 95.45% (3-sigma) 99.7 confidence ellipse

%% PLOT true map
if doTrueMap
    mfht = figure;
    figure(mfht)
    plot(groundTruth.path(1,:),groundTruth.path(2,:),'g','linewidth',2);
    hold on;
    plot(groundTruth.landmarkPos(1,:),groundTruth.landmarkPos(2,:),'*g');
    hold off;
    xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    axis equal; grid on;
    set(mfht,'Units','Inches');
    pos = get(mfht,'Position');
    set(mfht,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(strcat(baseDirectory,datafoldername(1:end-1),'-truemap'),'-depsc');
end

%% PLOT LFGO
if doLFGO
    
    lfgoResult = load(lfgofile); % load data
    lfgodat = lfgoResult.outdat;    
    
    mfh1 = figure;
    figure(mfh1)    
    plot(groundTruth.path(1,:),groundTruth.path(2,:),'g','linewidth',1);
    hold on;
    plot(initialGuessTraj(1,:),initialGuessTraj(2,:),'--k','linewidth',2);
    plot(lfgodat.estimatedPose(1,:), lfgodat.estimatedPose(2,:),'b','linewidth',2);
    %plot(groundTruth.landmarkPos(1,:),groundTruth.landmarkPos(2,:),'*g');
    %plot(lfgodat.estimatedFeats(1,:), lfgodat.estimatedFeats(2,:),'*b');
    
    pcovlfgo = [];    
    % plot feature error covariance
%     for i = 1:size(lfgodat.featCovariance,3)
%         ptemp = error_ellipse('C',lfgodat.featCovariance(:,:,i),'mu',lfgodat.estimatedFeats(:,i),'conf',confidence,'scale',1,'N', N);
%         pcovlfgo = [pcovlfgo ptemp];
%     end    
    %plot(pcovlfgo(1,:),pcovlfgo(2,:),'k');
    
    xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    title('\textbf{RFM-SLAM Estimate}','fontsize',14,'HorizontalAlignment', 'center','Interpreter','latex');
    axis equal; grid on;
    set(mfh1,'Units','Inches');
    pos = get(mfh1,'Position');
    set(mfh1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(strcat(baseDirectory,datafoldername(1:end-1),'-run',num2str(runToPlot),'-RFM'),'-depsc');
    
end

%% PLOT RFM-SLAM
if doRFMSLAM
    
    rfmslamResult = load(rfmslamfile); % load data
    rfmslamdat = rfmslamResult.outdat;    
    
    mfh1 = figure;
    figure(mfh1)    
    plot(groundTruth.path(1,:),groundTruth.path(2,:),'g','linewidth',1);
    hold on;
    plot(initialGuessTraj(1,:),initialGuessTraj(2,:),'--k','linewidth',2);
    plot(rfmslamdat.estimatedPose(1,:), rfmslamdat.estimatedPose(2,:),'b','linewidth',2);
    %plot(groundTruth.landmarkPos(1,:),groundTruth.landmarkPos(2,:),'*g');
    %plot(rfmslamdat.estimatedFeats(1,:), rfmslamdat.estimatedFeats(2,:),'*b');
    
    pcovRFMSLAM = [];    
    % plot feature error covariance
    for i = 1:size(rfmslamdat.featCovariance,3)
        ptemp = error_ellipse('C',rfmslamdat.featCovariance(:,:,i),'mu',rfmslamdat.estimatedFeats(:,i),'conf',confidence,'scale',1,'N', N);
        pcovRFMSLAM = [pcovRFMSLAM ptemp];
    end    
    %plot(pcovRFMSLAM(1,:),pcovRFMSLAM(2,:),'k');
    
    xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    title('\textbf{RFM-SLAM Estimate}','fontsize',14,'HorizontalAlignment', 'center','Interpreter','latex');
    axis equal; grid on;
    set(mfh1,'Units','Inches');
    pos = get(mfh1,'Position');
    set(mfh1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(strcat(baseDirectory,datafoldername(1:end-1),'-run',num2str(runToPlot),'-RFM'),'-depsc');
    
end

%% PLOT GTSAM
if doGTSAM
    
    gtsamResult = load(gtsamfile); % load data
    gtsamdat = gtsamResult.outdat;
   
    mfh2 = figure;
    figure(mfh2)
    plot(groundTruth.path(1,:),groundTruth.path(2,:),'g','linewidth',1);
    hold on;
    plot(initialGuessTraj(1,:),initialGuessTraj(2,:),'--k','linewidth',2);
    plot(gtsamdat.estimatedPose(1,:), gtsamdat.estimatedPose(2,:),'m','linewidth',2);    
    %plot(groundTruth.landmarkPos(1,:),groundTruth.landmarkPos(2,:),'*g');    
    %plot(gtsamdat.estimatedFeats(1,:), gtsamdat.estimatedFeats(2,:),'*m');
    
%     pcovGTSAM = [];    
%     % plot feature error covariance
%     for i = 1:size(gtsamdat.featCovariance,3)
%         ptemp = error_ellipse('C',gtsamdat.featCovariance(:,:,i),'mu',gtsamdat.estimatedFeats(:,i),'conf',confidence,'scale',1,'N', N);
%         pcovGTSAM = [pcovGTSAM ptemp];
%     end    
%     %plot(pcovGTSAM(1,:),pcovGTSAM(2,:),'m');
    
    xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
    title('\textbf{GTSAM Estimate}','fontsize',14,'HorizontalAlignment', 'center','Interpreter','latex');
    axis equal; grid on;
    set(mfh2,'Units','Inches');
    pos = get(mfh2,'Position');
    set(mfh2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(strcat(baseDirectory,datafoldername(1:end-1),'-run',num2str(runToPlot),'-GTSAM'),'-depsc');
end