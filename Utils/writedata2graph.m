function check = writedata2graph(t,knownHeading,odoData,obs,obsNoiseSigma,sigmaHeading, headingObs, filePath)
%This function writes the odometery data and the estimated trajectory into
% a file which can be fed to GTSAM as input
warning('There is a hard-coded precision value set for how many decimal values to print to file. Beware!')

%% Start writing graph file
fileName = 'simData.graph';
fName = strcat(filePath,fileName);
fileID = fopen(fName,'wt');  %21.06.2016

% If cannot open file return failure
if fileID == -1
    check = 0;
    warning('Could not open gtsam data output file');
    return;
end

% writing Initial Estimate of the robot trajectory (which is basically integration of odometery)
for i=1:t
    fprintf(fileID,'VERTEX2 %d %f %f %f\n',i,odoData.robotPose(:,i)');
end

% Writing odometry data as edges between successive poses
for i=1:t-1
    fprintf(fileID,'EDGE2 %d %d %3.8f %3.8f %3.8f %3.8f %3.8f %3.8f %3.8f %3.8f %3.8f\n',i,i+1,odoData.odo(:,i)',odoData.odoError(1,1:end,i)...
        ,odoData.odoError(2,2:end,i),odoData.odoError(3,3:end,i));
end

% Writing Range and Bearing measurements for the landmarks
for i=1:t    
    for j = 1:size(obs(i).rb,2)
        fprintf(fileID,'BR %d %d %3.6f %3.6f %3.6f %3.6f\n',...
             i,t+obs(i).rb(1,j),obs(i).rb(3,j),...
             obs(i).rb(2,j),obsNoiseSigma(2),obsNoiseSigma(1));
    end
end

% If heading is known
if knownHeading == 1        
    for h = 1:t       
        fprintf(fileID,'HD2 %d %3.8f %3.8f\n',h,headingObs(h),sigmaHeading); 
    end
end

fclose(fileID);

check = 1;
end