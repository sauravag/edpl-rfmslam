%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RFM-SLAM
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function slam_sim2d(runNumber, inpDat, odoNL, rbNL, varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D simulation for given environment
% Run robot in map through waypoints,
% write the ground truth, data to files.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Total Path Length = %f m \n',pathLength(inpDat.wp));

%% Setup parameters for simulation
global param;

param.doPLOT = 0;
param.plotFrequency = 5;
param.storeFrequency = 1; % We need this to be 1, because all sensed heading values (each pose) are required for GTSAM comparison
% Set sim parameters
param.odoNoiseLevelScale = odoNL; % [1,2,3,4] scale the motion odometery noises with this value
param.rbNoiseLevelScale = rbNL; % [1,2,3,4] scale the range bearing noises with this value
param.numSimLoops = 1; % number of loops robot will drive around trajectory
param.robot.maxVelocity = 5.0;
param.numSamples = 10;
param.simulator.dt = 0.1;
param.knownOrientation = 0; % if 1, heading is known if 0 heading is not known
param.headingUpdateInterval = 1; % time interval between heading updates
param.maxObservations = 1000;
param.minObservations = 50;
param.world.landmarks = struct();

% Put robot at initial point and set belief
param.robot.initialbelief.mean = zeros(3,1);
param.robot.initialbelief.covariance = diag([(0.01)^2, (0.01)^2, (0.05*pi/180)^2]);

% Create Structs to store data for GTSAM
odoData = struct('odo',[],'robotPose',[],'odoError',[]);
% odoData srores the odmetry data, its covariance matrix and robot expected
% poses based upon only odometry
obsData = struct();
% obsData stores the range and bearing data for landmarks with their IDs.
lmID = []; % It stores the full array of all the feature which were observed with their IDs.

% Put features into the world
for nf = 1:size(inpDat.lm,2)
    param.world.landmarkIDs(nf) = nf;
    param.world.landmarkPoses(:,nf) = inpDat.lm(:,nf);
end

%% Plotting setup
if param.doPLOT
    param.animationPlot = figure;
    plot(inpDat.lm(1,:),inpDat.lm(2,:),'b*')
    hold on; axis equal; axis tight
    plot(inpDat.wp(1,:),inpDat.wp(2,:), 'gs', inpDat.wp(1,:),inpDat.wp(2,:),'g.'); % plot waypoints
    xlabel('X (m)'); ylabel('Y (m)');
    set(param.animationPlot, 'name', 'PLUM-EKF Hybrid SLAM Simulator','WindowStyle', 'docked')
end

%% Initialize objects

% Initialize map
map = TwoDMap(varargin{1});

% If you have known features, insert them into the map
% map.mappedFeatureIDs = [param.world.landmarkIDs(1) param.world.landmarkIDs(20)];
% map.mappedFeaturePoses = [param.world.landmarkPoses(:,1) param.world.landmarkPoses(:,20)];

% create data association method
dataAssociationMethod = KnownDataAssociation();

% create sensors / obs models
orientationSensor = YawHeadingSensorModel();
obsModel = TwoDRangeBearingModel(map, param.world.landmarkIDs, param.world.landmarkPoses) ; % Init obs model

% create robot model
motionModel = TwoDBicycleRobot(); % Init motion model
bot = Robot(param.robot.initialbelief.mean, obsModel, motionModel); % Give sensor and motion model to robot

% create simulator, might be unecessary at this stage, helps in separation
sim = Simulator(bot); % Give sim the robot

%% Run looop
% filterNEES = [];
iwp = 1;
u = [0;0];
t = 0;
groundTruth = []; % to store actual robot traj
headingObs = [];
graphNodes = struct('idfs',[]);% store the ids seen at this node. 

while iwp ~= 0
    
    if t == 0
        u = zeros(2,1);
    else
        u(1) = param.robot.maxVelocity;
        [u(2),iwp] = motionModel.computeSteering( bot.state, inpDat.wp, iwp, u(2));
        if iwp==0 && param.numSimLoops > 1, iwp=1; param.numSimLoops= param.numSimLoops-1; end % perform loops: if final waypoint reached, go back to first
    end
    
    bot = sim.moveRobot(u); % move robot
    
    % add to ground truth
    groundTruth = [groundTruth, bot.state];
    
    % get the odometry readings and updated odometeric estimate, not that
    % odometery update has to be based on previously esimated odometery
    % state.
    if t == 0
        [odoVal, odoCov, odo_x_next] = sim.getOdometery(param.robot.initialbelief.mean, u);
        odoData.robotPose(:,t+1) = param.robot.initialbelief.mean; % at t=0, we give the known data
    else
        [odoVal, odoCov, odo_x_next] = sim.getOdometery(odoData.robotPose(:,t), u);
        odoData.odoError(:,:,t) = odoCov; % storing the odometry covariance or GTSAM
        % pose by odometry data
        odoData.odo(:,t) = odoVal;
        odoData.robotPose(:,t+1) = odo_x_next;
    end
    
    [z, idfs] = sim.getObservation(); % get observation   
    
    if runNumber ==1
        graphNodes(t+1).idfs = idfs;
    end
    
    %There are no duplicates (if I see id 20, it will put value 20 in position 20).
    lmID(idfs) = idfs; % Need this for post processing analysis
    
    % Storing the Observation in structure
    obsData(t+1).rb = [idfs;z];
    
    zOrientation = orientationSensor.getObservation(bot.state);
    
    headingObs = [headingObs, zOrientation];
    
    %fprintf('Time Step  = %u \n', t);
    
    if mod(t,param.plotFrequency) == 0 && param.doPLOT
        motionModel.draw(bot.state,[0;0;0]);
        obsModel.draw(bot.state, z);
        drawnow;
    end
    
    t = t+1; % t needs to be incremented before exiting
    
end


%% File Handling
check1 = writedata2graph(t,param.knownOrientation,odoData,obsData,bot.observationModel.sigma_b,orientationSensor.sigma_b,headingObs,varargin{1});

if check1
    fprintf('Graph write operation Successful \n');
else
    fprintf('Graph write Operation Failed \n');
end

% You don't want to write ground truth and compute edges for each run
% So you can replace nan with 1 to create an output otherwise
% leave it at nan
if runNumber == 1
    
    % Only build edges when orientation is not known
    if param.knownOrientation == 0        
        gEdges = buildEdges(graphNodes);
    else
        gEdges = [];
    end

    % By doing 'find' operation, we remove the zeros. The values are 0 where
    % the landmarks for that ID were not seen.
    check2 = writegroundtruth2file([varargin{1},'../'], groundTruth, lmID(find(lmID(:))), param.world.landmarkPoses, gEdges);
    
    if check2
        fprintf('Ground truth write operation Successful \n');
    else
        fprintf('Ground truth Write Operation Failed \n');
    end
end


end

function d = distance(p,q)
d = sqrt(sum((p-q).^2));
end

function L = pathLength(wp)
L = 0;
for i = 1:size(wp,2)-1
    L = L + distance(wp(:,i),wp(:,i+1));
end
end

function gEdges = buildEdges(graphNodes)
% This function generates the graph edges
% Very useful to speed up analysis

% get number of nodes.
N = length(graphNodes);

% get possible combinations
possibleEdges = nchoosek(1:N,2);

gEdges = [(1:N-1)' (2:N)']; %odometery edges

for e = 1:size(possibleEdges,1)
    fromNode = possibleEdges(e,1);
    toNode = possibleEdges(e,2);
    
    if abs(fromNode-toNode) < 2
        continue; % ignore odometery edges        
    end
    
    zFrom = graphNodes(fromNode).idfs;
    zTo = graphNodes(toNode).idfs;
    
    if size(zFrom,2) < 2 || size(zTo,2) < 2
        continue; % have to both see atleast 2 to have a constraint
    end
    
    [zints,~,~] = intersect(zFrom, zTo);
    
    % if less than 2 common
    if size(zints,2) < 2
        continue;
    end
    
    gEdges(end+1,:) = [fromNode, toNode];
    
end

end





